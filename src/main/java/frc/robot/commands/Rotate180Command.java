package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Rotate180Command extends RotateToTarget {
    public Rotate180Command(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
        super(drivetrain, currentPoseSupplier, compute180Rotation(currentPoseSupplier.get()));
    }

    private static Rotation2d compute180Rotation(Pose2d currentPose) {
        return currentPose.getRotation().plus(new Rotation2d(Degrees.of(180)));
    }
}
