package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.GameConstants;

/**
 * Rotates the robot to face the hub/goal.
 * 
 * <p>
 * This command uses the robot's current position and alliance color to
 * determine
 * the target hub location, calculates the angle to face that hub, and commands
 * the
 * drivetrain to rotate accordingly while maintaining position. The command
 * finishes
 * when the robot's heading matches the target angle.
 * 
 * <p>
 * Requires: {@link CommandSwerveDrivetrain}
 */
public class RotateToHubCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    /** The maximum allowable error in degrees */
    private final static double kToleranceDegrees = 0.5;
    /** The current error between the robot's heading and the target angle */
    private Angle errorAngle;

    private final DoublePublisher errorPub = NetworkTableInstance.getDefault()
            .getTable("rotateToHub")
            .getDoubleTopic("error")
            .publish();

    private final DoublePublisher currentPub = NetworkTableInstance.getDefault()
            .getTable("rotateToHub")
            .getDoubleTopic("current")
            .publish();

    private final DoublePublisher targetPub = NetworkTableInstance.getDefault()
            .getTable("rotateToHub")
            .getDoubleTopic("target")
            .publish();

    /**
     * Constructs a RotateToHubCommand.
     *
     * @param drivetrain the swerve drivetrain subsystem
     */
    public RotateToHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.errorAngle = Degrees.of(0);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d vectorToTarget = null;

        if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Blue) {
            vectorToTarget = GameConstants.blueHubLocation
                    .minus(currentPose.getTranslation());
        } else if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red) {
            vectorToTarget = GameConstants.redHubLocation
                    .minus(currentPose.getTranslation());
        }

        Rotation2d targetAngle = vectorToTarget.getAngle().minus(Rotation2d.fromRadians(Math.PI));
        this.errorAngle = Degrees.of(Math.abs(currentPose.getRotation().minus(targetAngle).getDegrees()));

        drivetrain.driveToPose(new Pose2d(currentPose.getX(), currentPose.getY(), targetAngle));
        errorPub.set(errorAngle.in(Degrees));
        currentPub.set(currentPose.getRotation().getDegrees());
        targetPub.set(targetAngle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return errorAngle.in(Degrees) < kToleranceDegrees;
    }
}
