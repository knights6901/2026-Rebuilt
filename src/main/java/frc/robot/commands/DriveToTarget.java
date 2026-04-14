package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.GameConstants;

public class DriveToTarget extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Supplier<SwerveRequest.FieldCentric> driverInput;

        // null = use driver input for that axis
        private final Supplier<Distance> targetX;
        private final Supplier<Distance> targetY;
        private final Supplier<Angle> targetTheta;

        // PID controllers for each axis, only used if the axis is being controlled by
        // the command
        private final PIDController xController = new PIDController(.5, 0, 0);
        private final PIDController yController = new PIDController(1, 0, 0);
        private final PIDController thetaController = new PIDController(2.5, 0, 0.0);

        private Angle thetaError;
        private Distance xError;
        private Distance yError;

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
                        Supplier<FieldCentric> driverInput,
                        Supplier<Distance> targetX,
                        Supplier<Distance> targetY,
                        Supplier<Angle> targetTheta) {
                this.drivetrain = drivetrain;
                this.driverInput = driverInput;

                this.targetX = targetX;
                this.targetY = targetY;
                this.targetTheta = targetTheta;

                addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
                xController.reset();
                yController.reset();
                thetaController.reset();
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                thetaController.setTolerance(Degrees.of(1).in(Radians));
        }

        @Override
        public void execute() {
                Pose2d currentPose = drivetrain.getPose();
                FieldCentric input = driverInput.get();

                double vX = targetX != null ? xController.calculate(currentPose.getX(), targetX.get().in(Meters))
                                : input.VelocityX;

                double vY = targetY != null ? yController.calculate(currentPose.getY(), targetY.get().in(Meters))
                                : input.VelocityY;

                double omega = targetTheta != null
                                ? thetaController.calculate(currentPose.getRotation().getRadians(),
                                                targetTheta.get().in(Radians))
                                : input.RotationalRate;

                drivetrain.setControl(input.withVelocityX(vX).withVelocityY(vY).withRotationalRate(omega));

                xError = targetX != null ? targetX.get().minus(currentPose.getMeasureX()) : Meters.of(0);
                yError = targetY != null ? targetY.get().minus(currentPose.getMeasureY()) : Meters.of(0);
                thetaError = targetTheta != null
                                ? targetTheta.get().minus(currentPose.getRotation().getMeasure())
                                : Degrees.of(0);

                thetaErrorPub.set(thetaError.in(Degrees));
                xErrorPub.set(xError.in(Meters));
                yErrorPub.set(yError.in(Meters));
        }

        @Override
        public boolean isFinished() {
                boolean xComplete = targetX == null || xController.atSetpoint();
                boolean yComplete = targetY == null || yController.atSetpoint();
                boolean thetaComplete = targetTheta == null || thetaController.atSetpoint();

                return xComplete && yComplete && thetaComplete;
        }

        /** Rotates the robot to the alliance's hub. */
        public static Command rotateToHub(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInputSupplier) {
                return new DriveToTarget(
                                drivetrain,
                                driverInputSupplier,
                                null,
                                null,
                                () -> {
                                        Translation2d current = drivetrain.getPose().getTranslation();
                                        Translation2d hub = GameConstants.getHubLocation();

                                        double targetAngle = Math.atan2(
                                                        hub.getY() - current.getY(),
                                                        hub.getX() - current.getX());

                                        return Radians.of(targetAngle + Math.PI);
                                });
        }

        /** Rotates the robot by 180 degrees. */
        public static Command rotateBy180(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInputSupplier) {
                Angle[] target = { null };

                return Commands.runOnce(
                                () -> target[0] = drivetrain.getPose().getRotation().getMeasure().plus(Degrees.of(180)))
                                .andThen(new DriveToTarget(
                                                drivetrain, driverInputSupplier,
                                                null, null,
                                                () -> target[0]));
        }

        /** Drives the robot to be aligned with the alliance's closer trench. */
        public static Command alignToTrench(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInputSupplier) {

                Supplier<Distance> targetY = () -> {
                        Distance currentY = drivetrain.getPose().getMeasureY();
                        double toLeft = currentY.minus(GameConstants.BlueLeftTrenchY).abs(Meters);
                        double toRight = currentY.minus(GameConstants.BlueRightTrenchY).abs(Meters);
                        return toLeft < toRight ? GameConstants.BlueLeftTrenchY : GameConstants.BlueRightTrenchY;
                };

                Supplier<Angle> targetTheta = () -> {
                        double theta = ((drivetrain.getPose().getRotation().getDegrees() % 360) + 360) % 360;
                        if (theta > 180)
                                theta -= 360;
                        return Degrees.of(Math.abs(theta) < 90 ? 0 : 180);
                };

                return new DriveToTarget(drivetrain, driverInputSupplier,
                                null,
                                targetY,
                                targetTheta);
        }
}