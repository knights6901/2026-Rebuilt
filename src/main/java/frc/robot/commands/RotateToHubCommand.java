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

public class RotateToHubCommand extends RotateToTarget {
    public RotateToHubCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        super(drivetrain, currentPoseSupplier, () -> computeHubRotation(currentPoseSupplier.get()), Degrees.of(2));
    }

    private static Rotation2d computeHubRotation(Pose2d currentPose) {
        Translation2d targetHub = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? GameConstants.blueHubLocation
                : GameConstants.redHubLocation;

        return targetHub.minus(currentPose.getTranslation()).getAngle();
    }
}
