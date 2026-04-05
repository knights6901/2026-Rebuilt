package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.TunerConstants.TunerSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.math.controller.PIDController;

/**
 * Swerve drivetrain subsystem built on the CTRE Phoenix 6
 * {@code SwerveDrivetrain}.
 *
 * <p>
 * Handles field-centric driving, PathPlanner autonomous path following, SysId
 * characterization routines, vision-measurement fusion, operator-perspective
 * correction, and simulation support.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric();

    private final Field2d m_field = new Field2d();
    private final StructPublisher<Pose3d> m_posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose3d.struct).publish();

    private final DoublePublisher m_CanUtilizationPublisher = NetworkTableInstance.getDefault()
            .getTable("Can")
            .getDoubleTopic("Canivore Utilization")
            .publish();

    private CANBusStatus status = TunerConstants.kCANBus.getStatus();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // PID velocity controllers
    private final PIDController xController = new PIDController(.5, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController thetaController = new PIDController(5, 0, 0.0);

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** SysId routine for characterizing drive-motor translation gains. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /** SysId routine for characterizing steer-motor gains. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /**
     * SysId routine for characterizing rotational gains, used by
     * {@code FieldCentricFacingAngle}'s heading controller.
     *
     * @see com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Creates the swerve drivetrain with default odometry frequency.
     *
     * @param drivetrainConstants drivetrain-wide constants
     * @param modules             constants for each swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("Field", m_field);

        configureAutoBuilder();
    }

    /**
     * Creates the swerve drivetrain with a custom odometry update frequency.
     *
     * @param drivetrainConstants     drivetrain-wide constants
     * @param odometryUpdateFrequency odometry loop frequency in Hz (0 for default)
     * @param modules                 constants for each swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("Field", m_field);

        configureAutoBuilder();
    }

    /**
     * Creates the swerve drivetrain with custom odometry frequency and
     * standard-deviation matrices for odometry and vision.
     *
     * @param drivetrainConstants       drivetrain-wide constants
     * @param odometryUpdateFrequency   odometry loop frequency in Hz (0 for
     *                                  default)
     * @param odometryStandardDeviation standard deviations for odometry [x, y, θ]ᵀ
     *                                  (meters/radians)
     * @param visionStandardDeviation   standard deviations for vision [x, y, θ]ᵀ
     *                                  (meters/radians)
     * @param modules                   constants for each swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("Field", m_field);

        configureAutoBuilder();
    }

    /**
     * Drives the robot toward a target pose using PID controllers on
     * x, y, and heading. Intended to be called repeatedly (e.g. in
     * {@code execute()}).
     *
     * @param targetPose the field-relative pose to drive toward
     * @param controllersToUse array of booleans indicating which controllers to use:
     * 
     * update 4/3/2026 --> should not override driver completely now
     */
    public void driveToPose(Pose2d currentPose, Pose2d targetPose, boolean[] controllersToUse, SwerveRequest.FieldCentric driverInput) {
        xController.setTolerance(0.5);
        yController.setTolerance(0.5);
        thetaController.setTolerance(Math.PI / 18);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_fieldCentricRequest.VelocityX = driverInput.VelocityX;
        m_fieldCentricRequest.VelocityY = driverInput.VelocityY;
        m_fieldCentricRequest.RotationalRate = driverInput.RotationalRate;

        if (controllersToUse[0]) {
            m_fieldCentricRequest.VelocityX = -xController.calculate(currentPose.getX(), targetPose.getX());
        }
        if (controllersToUse[1]) {;
            m_fieldCentricRequest.VelocityY = -yController.calculate(currentPose.getY(), targetPose.getY());
        }
        if (controllersToUse[2]) {
            m_fieldCentricRequest.RotationalRate = thetaController.calculate(currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians());
        }

        setControl(m_fieldCentricRequest);
    }

    // public void rotateToPose(Pose2d currentPose, Pose2d targetPose) {
    //     thetaController.setTolerance(.01);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     double thetaVel = thetaController.calculate(
    //             currentPose.getRotation().getRadians(),
    //             targetPose.getRotation().getRadians());

    //     setControl(new SwerveRequest.FieldCentric()
    //             .withRotationalDeadband(0.1)
    //             .withRotationalRate(thetaVel));
    // }

    public void resetPIDControllers() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    () -> getState().Pose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())),
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            DriverStation.reportError("Failed to load path planner", e.getStackTrace());
        }
    }

    public boolean hasAppliedOperatorPerspective() {
        return m_hasAppliedOperatorPerspective;
    }

    /**
     * Returns a command that continuously applies a swerve request to the
     * drivetrain.
     *
     * @param request supplier providing the {@link SwerveRequest} each iteration
     * @return command that runs until interrupted
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * Runs the SysId quasistatic test in the given direction for the active
     * routine.
     *
     * @param direction direction of the quasistatic ramp
     * @return command that executes the test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId dynamic (step-voltage) test in the given direction for the
     * active routine.
     *
     * @param direction direction of the dynamic step
     * @return command that executes the test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        m_CanUtilizationPublisher.set(status.BusUtilization);

        m_field.setRobotPose(this.getState().Pose);

        if (RobotBase.isSimulation()) {
            Pose3d currentPose3d = new Pose3d(this.getState().Pose);
            m_posePublisher.set(currentPose3d);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * {@inheritDoc}
     *
     * <p>
     * Converts the FPGA timestamp to the current timebase before forwarding
     * to the superclass.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * {@inheritDoc}
     *
     * <p>
     * Converts the FPGA timestamp to the current timebase and forwards the
     * custom standard-deviation matrix to the superclass.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * {@inheritDoc}
     *
     * <p>
     * Converts the FPGA timestamp to the current timebase before sampling.
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Returns the robot's current field-relative pose from the odometry state.
     *
     * @return the current {@link Pose2d}
     */
    public Pose2d getPose() {
        return this.getState().Pose;
    }
}
