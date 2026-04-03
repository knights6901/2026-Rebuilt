package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final Supplier<Pose2d> currentPoseSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;

    private final boolean[] controllersToUse;
    private final Supplier<SwerveRequest.FieldCentric> driverInput;

    public final Angle ThetaErrorTolerance = Degrees.of(10);
    public final Distance DisplacementErrorTolerance = Meters.of(0.5);

    private Angle thetaError;
    private Distance xError;
    private Distance yError;

    private boolean isCompleted;

    private final DoublePublisher thetaErrorPub = NetworkTableInstance.getDefault()
            .getTable("DriveToTarget")
            .getDoubleTopic("Error (theta)")
            .publish();

    private final DoublePublisher xErrorPub = NetworkTableInstance.getDefault()
            .getTable("DriveToTarget")
            .getDoubleTopic("Error (x)")
            .publish();

    private final DoublePublisher yErrorPub = NetworkTableInstance.getDefault()
            .getTable("DriveToTarget")
            .getDoubleTopic("Error (y)")
            .publish();

    public DriveToTarget(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> currentPoseSupplier,
            Supplier<Pose2d> targetPoseSupplier,
            boolean[] controllersToUse,
            Supplier<SwerveRequest.FieldCentric> driverInput) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        this.controllersToUse = controllersToUse;
        this.driverInput = driverInput;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();
        drivetrain.resetPIDControllers();

        isCompleted = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();


        thetaError = currentPose.getRotation().minus(targetPose.getRotation()).getMeasure();
        xError = Meters.of(currentPose.getTranslation().getX() - targetPose.getTranslation().getX());
        yError = Meters.of(currentPose.getTranslation().getY() - targetPose.getTranslation().getY());

        isCompleted = (!controllersToUse[0] || xError.abs(Meters) <= DisplacementErrorTolerance.in(Meters))
           && (!controllersToUse[1] || yError.abs(Meters) <= DisplacementErrorTolerance.in(Meters))
           && (!controllersToUse[2] || thetaError.abs(Degrees) <= ThetaErrorTolerance.in(Degrees));
        
        if (!isCompleted) {
            drivetrain.driveToPose(currentPose, targetPose, controllersToUse, driverInput.get());
        }

        thetaErrorPub.set(thetaError.in(Degrees));
        xErrorPub.set(xError.in(Meters));
        yErrorPub.set(yError.in(Meters));
    }

    @Override
    public boolean isFinished() {
        return isCompleted;
    }
}
