package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class AlignToTrench extends DriveToTarget {
    
    public AlignToTrench(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> currentPoseSupplier,
            Supplier<SwerveRequest.FieldCentric> driverInputSupplier) {
        super(
                drivetrain,
                currentPoseSupplier, () -> getTrenchAlignedPose(currentPoseSupplier.get()),
                new boolean[]{false, true, false},
                driverInputSupplier);
    }

    private static Pose2d getTrenchAlignedPose(Pose2d currentPose) {
        double closerX = Math.abs(currentPose.getX() - 0.634) < Math.abs(currentPose.getX() - 7.435)
                ? 0.634
                : 7.435;

        return new Pose2d(closerX, currentPose.getY(), currentPose.getRotation());
    }
}