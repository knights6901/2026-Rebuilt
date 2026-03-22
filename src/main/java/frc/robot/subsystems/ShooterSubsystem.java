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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Telemetry;
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

    private final DoublePublisher leftVelocityPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("LeftVelocity")
            .publish();

    private final DoublePublisher rightVelocityPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("RightVelocity")
            .publish();

    private AngularVelocity shootRPS = ShooterConstants.DefaultRPS;

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
    }

    /**
     * Spins the flywheel at the specified angular velocity, typically used
     * for auto-aim shots where the velocity is computed from target distance.
     *
     * @param rps target angular velocity
     */
    public void shoot(AngularVelocity rps) {
        m_motorRight.setControl(m_request.withVelocity(rps));

    }

    /** Stops the flywheel by applying neutral output to both motors. */
    public void stop() {
        m_motorRight.setControl(new NeutralOut());
    }

    public void increaseShootRPS() {
        shootRPS = shootRPS.plus(RotationsPerSecond.of(1));
    }

    public void decreaseShootRPS() {
        shootRPS = shootRPS.minus(RotationsPerSecond.of(1));
    }

    public AngularVelocity getShootRPS() {
        return shootRPS;
    }

    /**
     * Publishes a 3D trajectory to NetworkTables for dashboard visualization.
     *
     * <p>
     * Computes a parabolic ball path by combining the robot's field-relative
     * velocity with the shot's launch velocity, then samples positions along the
     * arc at fixed time steps.
     *
     * @param robotPose          current field-relative pose of the robot
     * @param v0_mag             launch speed magnitude in m/s
     * @param launchAngleDegrees angle above horizontal at which the ball is
     *                           launched
     */
    public void updateShotVisualization(Pose2d robotPose, double v0_mag, double launchAngleDegrees) {
        // 1. Fetch robot pose (The starting point)
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // placeholders cuz i can't fetch them right now
        double robotVx = Telemetry.currentSpeeds.vxMetersPerSecond;
        double robotVy = Telemetry.currentSpeeds.vyMetersPerSecond;

        // 2. Calculate the Shot's initial velocity (without robot movement)
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double headingRad = robotPose.getRotation().getRadians();

        double shotVx = v0_mag * Math.cos(launchAngleRad) * Math.cos(headingRad);
        double shotVy = v0_mag * Math.cos(launchAngleRad) * Math.sin(headingRad);
        double shotVz = v0_mag * Math.sin(launchAngleRad);

        // 3. Combine them!
        double totalVx = shotVx + robotVx;
        double totalVy = shotVy + robotVy;
        double totalVz = shotVz;

        // 3. Create a list of Poses (the "streak")
        int samples = 20; // How many points to draw
        double dt = 0.1; // Time step between points (0.1 seconds)
        Pose3d[] trajectory = new Pose3d[samples];

        for (int i = 0; i < samples; i++) {
            double t = i * dt;

            // 4. Basic Kinematics (x = x0 + vt, z = z0 + vt - 0.5gt^2)
            double currX = robotX + (totalVx * t);
            double currY = robotY + (totalVy * t);
            double currZ = 0.5 + (totalVz * t) - (0.5 * 9.81 * t * t); // Starting 0.5m high

            // Clamp Z so it doesn't go through the floor in the visualizer
            if (currZ < 0)
                currZ = 0;

            // Create the Pose3d Object
            trajectory[i] = new Pose3d(currX, currY, currZ, new Rotation3d());
        }

        // 5. Upload to Network Tables (Using the modern Struct approach)
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
                .publish()
                .set(trajectory);
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
        double dx = groundDistance.in(Meters);
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
     * Clears the dashboard trajectory visualization by publishing an empty pose
     * array.
     */
    public void clearTrajectory() {
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
                .publish()
                .set(new Pose3d[0]);
    }

    @Override
    public void periodic() {
        // Publish current velocities for telemetry
        rightVelocityPub.set(m_motorRight.getVelocity().getValueAsDouble());
        leftVelocityPub.set(m_motorLeft.getVelocity().getValueAsDouble());
    }
}
