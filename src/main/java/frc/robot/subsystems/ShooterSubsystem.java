package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_motorRight = new TalonFX(RightMotorId, "rio");
    private final TalonFX m_motorLeft = new TalonFX(LeftMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    /// Initializes the shooter subsystem.
    public ShooterSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_motorRight.getConfigurator().apply(m_motorConfig);
        m_motorLeft.getConfigurator().apply(m_motorConfig);

        m_motorLeft.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
    }

    /// Shoots with a variable RPS based on the input axis value (e.g., from a
    /// trigger).
    public void shoot(double axis) {
        m_motorRight.setControl(m_request.withVelocity(ShootRPS.times(axis)));
    }

    /// Shoots with a preset RPS defined in Constants.
    public void shoot() {

        m_motorRight.setControl(m_request.withVelocity(ShootRPS));
    }

    /// Shoots with a specified RPS.
    public void shoot(int rps) {
        m_motorRight.setControl(m_request.withVelocity(rps));
    }

    /// Shoots with a calculated RPS for auto-aiming.
    public void shoot(AngularVelocity rps) {
        m_motorRight.setControl(m_request.withVelocity(rps));
    }

    // Disables both motors by setting their power to 0.
    public void stop() {
        m_motorRight.setControl(new NeutralOut());
    }

    /// Updates the shot visualization in the dashboard by calculating the
    /// trajectory of the ball based on the robot's current pose, velocity, and the
    /// launch angle.
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

    /// Calculates the required RPS to hit the target based on the given pitch and
    /// horizontal distance to the target.
    ///
    /// @param groundDistance The horizontal distance from the robot to the target.
    public AngularVelocity calculateRPS(Distance groundDistance) {
        double dx = groundDistance.in(Meters);
        double dy = HubTargetHeight.minus(BallExtakeHeight).in(Meters);

        double gVal = G.in(MetersPerSecondPerSecond);
        double pitchRad = Pitch.in(Radians);

        double velocity = Math.sqrt(
                (gVal * dx * dx) /
                        (2 * Math.pow(Math.cos(pitchRad), 2) *
                                (dx * Math.tan(pitchRad) - dy)));

        double rps = velocity / (2 * Math.PI * 0.051);

        return RotationsPerSecond.of(DampingCoefficient * rps);
    }

    /// Clears the trajectory visualization by publishing an empty array to the same
    /// Network Table topic.
    public void clearTrajectory() {
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
                .publish()
                .set(new Pose3d[0]);
    }
}
