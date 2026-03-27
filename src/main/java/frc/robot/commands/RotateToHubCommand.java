package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.GameConstants;

public class RotateToHubCommand extends DriveToTarget {
    public RotateToHubCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        super(drivetrain, currentPoseSupplier, () -> {
            Pose2d currentPose = currentPoseSupplier.get();

            Rotation2d rotation = computeHubRotation(currentPose);
            Translation2d translation = currentPose.getTranslation();

            return new Pose2d(translation, rotation);
        });
    }

    private static Rotation2d computeHubRotation(Pose2d currentPose) {
        Translation2d targetHub = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? GameConstants.BlueHubLocation
                : GameConstants.RedHubLocation;

        return targetHub.minus(currentPose.getTranslation()).getAngle().plus(new Rotation2d(Degrees.of(180)));
    }
}
