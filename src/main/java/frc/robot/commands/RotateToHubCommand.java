package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.GameConstants;

public class RotateToHubCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;

    public RotateToHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

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

        // just delete the minus if the robot turns out to face the wrong way
        Rotation2d targetAngle = vectorToTarget.getAngle().minus(Rotation2d.fromRadians(Math.PI));
        drivetrain.driveToPose(new Pose2d(currentPose.getX(), currentPose.getY(),
                        targetAngle));
    }

    @Override
    public boolean isFinished() {
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
        double errorDegrees = Math.abs(currentPose.getRotation().minus(targetAngle).getDegrees());
        return errorDegrees < 1.0;
    }
}
