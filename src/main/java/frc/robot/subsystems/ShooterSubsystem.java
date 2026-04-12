package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem controlling the dual-motor shooter flywheel.
 *
 * <p>
 * The left motor follows the right motor in the opposed direction. Provides
 * multiple shoot overloads for manual, automatic, and auto-aim velocity
 * control, as well as trajectory visualization and ballistics calculations
 * for distance-based shot speed.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_motorRight = new TalonFX(ShooterConstants.RightMotorId, new CANBus("rio"));
    private final TalonFX m_motorLeft = new TalonFX(ShooterConstants.LeftMotorId, new CANBus("rio"));
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    /** The possible states of the shooter mechanism. */
    public static enum ShooterState {
        OFF,
        AUTOHUB,
        AUTOPASS,
        PRIMING,
        MANUAL
    }

    private final StringPublisher shooterStatePub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getStringTopic("Shooter?")
            .publish();

    private final DoublePublisher targetRPSPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("TargetRPS")
            .publish();

    private final DoublePublisher actualRPSPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("ActualRPS")
            .publish();

    public ShooterState shooterState = ShooterState.OFF;

    private AngularVelocity shootRPS = ShooterConstants.DefaultRPS;

    private AngularVelocity targetRPS;

    /**
     * Configures both shooter motors with PID gains from constants and sets the
     * left motor to follow the right motor in the opposed direction.
     */
    public ShooterSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterConstants.Gains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_motorRight.getConfigurator().apply(m_motorConfig);
        m_motorLeft.getConfigurator().apply(m_motorConfig);

        m_motorLeft.setControl(new Follower(ShooterConstants.RightMotorId, MotorAlignmentValue.Opposed));
    }

    public void shoot() {
        m_motorRight.setControl(m_request.withVelocity(shootRPS));
        targetRPS = shootRPS;
    }

    /**
     * Spins the flywheel at the specified angular velocity, typically used
     * for auto-aim shots where the velocity is computed from target distance.
     *
     * @param rps target angular velocity
     */
    public void shoot(AngularVelocity rps) {
        m_motorRight.setControl(m_request.withVelocity(rps));
        targetRPS = rps;
    }

    /** Stops the flywheel by applying neutral output to both motors. */
    public void stop() {
        m_motorRight.setControl(new NeutralOut());
        shooterState = ShooterState.OFF;
        targetRPS = RotationsPerSecond.of(0);
    }

    public void increaseShootRPS() {
        shootRPS = shootRPS.plus(RotationsPerSecond.of(1));
    }

    public void decreaseShootRPS() {
        shootRPS = shootRPS.minus(RotationsPerSecond.of(1));
    }

    /**
     * Calculates the required flywheel angular velocity to hit the hub target at
     * the given horizontal distance, using projectile kinematics with a fixed
     * launch pitch angle.
     *
     * @param groundDistance horizontal distance from the robot to the target
     * @return the flywheel angular velocity needed to reach the target
     */
    public AngularVelocity calculateRPS(Distance groundDistance) {
        double dx = groundDistance.minus(ShooterConstants.CenterToShooter).in(Meters);
        double dy = ShooterConstants.HubTargetHeight.minus(ShooterConstants.BallExtakeHeight).in(Meters);

        double gVal = ShooterConstants.G.in(MetersPerSecondPerSecond);
        double pitchRad = ShooterConstants.Pitch.in(Radians);

        double velocity = Math.sqrt(
                (gVal * dx * dx) /
                        (2 * Math.pow(Math.cos(pitchRad), 2) *
                                (dx * Math.tan(pitchRad) - dy)));

        double rps = velocity / (2 * Math.PI * 0.051);

        return RotationsPerSecond.of(ShooterConstants.DampingCoefficient * rps);
    }

    /**
     * Calculates the required flywheel angular velocity to hit the hub target at
     * the given horizontal distance, using projectile kinematics with a fixed
     * launch pitch angle.
     *
     * @param groundDistance horizontal distance from the robot to the target
     * @param targetHeight   vertical height of the target from the floor
     * @return the flywheel angular velocity needed to reach the target
     */
    public AngularVelocity calculateRPS(Distance groundDistance, Distance targetHeight) {
        double dx = groundDistance.in(Meters);
        double dy = targetHeight.minus(ShooterConstants.BallExtakeHeight).in(Meters);

        double gVal = ShooterConstants.G.in(MetersPerSecondPerSecond);
        double pitchRad = ShooterConstants.Pitch.in(Radians);

        double velocity = Math.sqrt(
                (gVal * dx * dx) /
                        (2 * Math.pow(Math.cos(pitchRad), 2) *
                                (dx * Math.tan(pitchRad) - dy)));

        double rps = velocity / (2 * Math.PI * 0.051);

        return RotationsPerSecond.of(ShooterConstants.DampingCoefficient * rps);
    }

    public AngularVelocity getShootRPS() {
        return shootRPS;
    }

    public AngularVelocity getAPManualRPS(double axisInput) {
        return ShooterConstants.MaxRPS.times(axisInput);
    }

    /**
     * Clears the dashboard trajectory visualization by publishing an empty pose
     * array.
     */
    // public void clearTrajectory() {
    // NetworkTableInstance.getDefault()
    // .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
    // .publish()
    // .set(new Pose3d[0]);
    // }

    @Override
    public void periodic() {
        // Publish current velocities for telemetry
        if (shooterState != ShooterState.MANUAL) {
            shooterStatePub.set(shooterState.toString());
        } else {
            shooterStatePub.set(shootRPS.in(RotationsPerSecond) + " (MANUAL)");
        }

        actualRPSPub.set(m_motorLeft.getVelocity().getValueAsDouble());

        targetRPSPub.set(targetRPS != null ? targetRPS.in(RotationsPerSecond) : 0);
    }
}
