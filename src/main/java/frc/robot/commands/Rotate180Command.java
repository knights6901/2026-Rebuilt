package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Rotate180Command extends DriveToTarget {
    public Rotate180Command(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        super(drivetrain, currentPoseSupplier, () -> {
            Pose2d currentPose = currentPoseSupplier.get();

            Rotation2d rotation = compute180Rotation(currentPose);
            Translation2d translation = currentPose.getTranslation();

            return new Pose2d(translation, rotation);
        });
    }

    private static Rotation2d compute180Rotation(Pose2d currentPose) {
        return currentPose.getRotation().plus(new Rotation2d(Degrees.of(180)));
    }
}
