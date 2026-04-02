package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTrench extends DriveToTarget {
    public AlignToTrench(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> currentPoseSupplier) {
        super(
                drivetrain,
                currentPoseSupplier, () -> getTrenchAlignedPose(currentPoseSupplier.get()));
    }

    private static Pose2d getTrenchAlignedPose(Pose2d currentPose) {
        // TODO: actually figure this out

        return currentPose;
    }
}