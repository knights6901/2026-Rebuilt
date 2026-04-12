package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.EnumSet;
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
        /**
         * The axes that can be controlled by this command. If an axis is not included,
         * the driver input will be used for that axis instead.
         */
        public enum Axis {
                X, Y, THETA
        }

        private final CommandSwerveDrivetrain drivetrain;

        private final Supplier<Pose2d> currentPoseSupplier;
        private final Supplier<Pose2d> targetPoseSupplier;
        private Pose2d targetPose;

        private final EnumSet<Axis> controlledAxes;
        private final Supplier<SwerveRequest.FieldCentric> driverInput;

        public final Angle ThetaErrorTolerance = Degrees.of(5);
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
                        Supplier<SwerveRequest.FieldCentric> driverInput,
                        Axis... controlledAxes) {
                this.drivetrain = drivetrain;
                this.currentPoseSupplier = currentPoseSupplier;
                this.targetPoseSupplier = targetPoseSupplier;
                this.driverInput = driverInput;
                this.controlledAxes = controlledAxes.length > 0
                                ? EnumSet.copyOf(Arrays.asList(controlledAxes))
                                : EnumSet.noneOf(Axis.class);

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

                isCompleted = (!controlledAxes.contains(Axis.X)
                                || xError.abs(Meters) <= DisplacementErrorTolerance.in(Meters))
                                && (!controlledAxes.contains(Axis.Y)
                                                || yError.abs(Meters) <= DisplacementErrorTolerance.in(Meters))
                                && (!controlledAxes.contains(Axis.THETA)
                                                || thetaError.abs(Degrees) <= ThetaErrorTolerance.in(Degrees));

                if (!isCompleted) {
                        SwerveRequest.FieldCentric input = driverInput.get();

                        double vx = controlledAxes.contains(Axis.X)
                                        ? drivetrain.calculateXVelocity(currentPose.getX(), targetPose.getX())
                                        : input.VelocityX;

                        double vy = controlledAxes.contains(Axis.Y)
                                        ? drivetrain.calculateYVelocity(currentPose.getY(), targetPose.getY())
                                        : input.VelocityY;

                        double omega = controlledAxes.contains(Axis.THETA)
                                        ? drivetrain.calculateRotationalRate(
                                                        currentPose.getRotation().getRadians(),
                                                        targetPose.getRotation().getRadians())
                                        : input.RotationalRate;

                        drivetrain.setControl(input.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
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
