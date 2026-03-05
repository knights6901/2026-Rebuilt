package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motorRight = new TalonFX(RightMotorId, "rio");
    private final TalonFX motorLeft = new TalonFX(LeftMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorRight.getConfigurator().apply(m_motorConfig);
        motorLeft.getConfigurator().apply(m_motorConfig);

        motorLeft.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
    }

    public void shoot(double axis) {
        motorRight.setControl(m_request.withVelocity(axis * shootRPS));
    }

    public void shoot() {
        motorRight.setControl(m_request.withVelocity(shootRPS));
    }

    public void shoot(int rps) {
        motorRight.setControl(m_request.withVelocity(rps));
    }

    public void shootWithAutoAim(double calcRPS) {
        motorRight.setControl(m_request.withVelocity(calcRPS));
    }

    // Disables both motors by setting their power to 0.
    public void stop() {
        motorRight.setControl(m_request.withVelocity(0));
    }

    public void updateShotVisualization(Pose2d robotPose, double v0_mag, double launchAngleDegrees) {
        // works but is scuffed, also it's not 100% AI anymore (i added some stuff +
        // made it work while moving yay)

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

    public double calculateRPS(double pitch, double groundDis) {
        // a mix of rao's desmos thing and conversions to rps. no clue how well it will
        // work bc load but can be tweaked a lot as needed

        // if watchdog timeout save cos(pitch) as a constant and use that

        double rps = 0;
        rps = (Math.sqrt((g * groundDis * groundDis) / (2 * Math.cos(pitch) * Math.cos(pitch)
                * (groundDis * Math.tan(pitch) - (vertDis - ballExtakeHeight))))) / (2 * Math.PI * 0.051);
        return (scaling * rps);
    }

    public void clearTrajectory() {
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
                .publish()
                .set(new Pose3d[0]);
    }
}
