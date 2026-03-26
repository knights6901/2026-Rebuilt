package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final Supplier<Pose2d> currentPoseSupplier;
    private final Supplier<Rotation2d> targetRotationSupplier;
    private Rotation2d targetRotation;

    private final Angle errorTolerance;
    private Angle error;

    private final DoublePublisher currentPub = NetworkTableInstance.getDefault()
            .getTable("RotateToPose")
            .getDoubleTopic("Current Angle")
            .publish();

    private final DoublePublisher targetPub = NetworkTableInstance.getDefault()
            .getTable("RotateToPose")
            .getDoubleTopic("Target Angle")
            .publish();

    private final DoublePublisher errorPub = NetworkTableInstance.getDefault()
            .getTable("RotateToPose")
            .getDoubleTopic("Error")
            .publish();

    public RotateToTarget(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier,
            Supplier<Rotation2d> targetRotationSupplier, Angle errorTolerance) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.targetRotationSupplier = targetRotationSupplier;
        this.errorTolerance = errorTolerance;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetRotation = targetRotationSupplier.get();
        targetPub.set(targetRotation.getDegrees());

        drivetrain.resetPIDControllers();
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();
        error = currentPose.getRotation().minus(targetRotation).getMeasure();

        drivetrain.rotateToPose(currentPose, new Pose2d(currentPose.getTranslation(), targetRotation));

        errorPub.set(error.in(Degrees));
        currentPub.set(currentPoseSupplier.get().getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return error.abs(Degrees) <= errorTolerance.in(Degrees);
    }
}
