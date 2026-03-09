// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class RobotContainer {
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConstants.MaxSpeed.times(ControllerConstants.kDeadband))
                        .withRotationalDeadband(DrivetrainConstants.MaxAngularRate.times(ControllerConstants.kDeadband))
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
        private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        // private final IndexerSubsystem indexer = new IndexerSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final SlapdownSubsystem slapdown = new SlapdownSubsystem();

        public RobotContainer() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        private void configureDriverBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> getDriverInput()));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driver.b().whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Point the wheels towards the hub when holding y.
                driver.y().whileTrue(new RunCommand(() -> {
                        Pose2d currentPose = drivetrain.getState().Pose;
                        Translation2d vectorToTarget = null;

                        if (DriverStation.getAlliance().isPresent() &&
                                        DriverStation.getAlliance().get() == Alliance.Blue) {
                                vectorToTarget = gameConstants.blueHubLocation
                                                .minus(currentPose.getTranslation());
                        } else if (DriverStation.getAlliance().isPresent() &&
                                        DriverStation.getAlliance().get() == Alliance.Red) {
                                vectorToTarget = gameConstants.redHubLocation
                                                .minus(currentPose.getTranslation());
                        }

                        Rotation2d targetAngle = vectorToTarget.getAngle();
                        drivetrain.driveToPose(new Pose2d(currentPose.getX(), currentPose.getY(),
                                        targetAngle));
                }));

                if (Robot.isSimulation()) {
                        driver.x().onTrue(new InstantCommand(() -> {
                                shooter.updateShotVisualization(drivetrain.getPose(), 7, 60);
                        })).onFalse(new InstantCommand(() -> shooter.clearTrajectory()));
                }

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public void configureOperatorBindings() {
                operator.rightBumper().whileTrue(new RunCommand(() -> {
                        Pose2d currentPose = drivetrain.getState().Pose;
                        Translation2d hubLocation = (DriverStation.getAlliance().get() == Alliance.Blue)
                                        ? gameConstants.blueHubLocation
                                        : gameConstants.redHubLocation;

                        Distance shotGroundDistance = Meters
                                        .of(currentPose.getTranslation().getDistance(hubLocation));

                        shooter.shootWithAutoAim(shooter.calculateRPS(shotGroundDistance));
                }));

                operator.rightTrigger().whileFalse(new InstantCommand(() -> {
                        if (!operator.rightBumper().getAsBoolean()) {
                                shooter.stop();
                        }
                }));

                operator.a().whileTrue(new InstantCommand(() -> {
                        shooter.shoot();
                }));

                operator.a().whileFalse(new InstantCommand(() -> {
                        shooter.stop();
                }));

                operator.y().whileTrue(new InstantCommand(() -> {
                }));

                operator.y().whileFalse(new InstantCommand(() -> {
                        shooter.stop();
                }));
        }

        // Generates the command request for moving the drive train based on the current
        // controller input.
        public FieldCentric getDriverInput() {
                return drive
                                .withVelocityX(DrivetrainConstants.MaxSpeed.times(-driver.getLeftY()))
                                .withVelocityY(DrivetrainConstants.MaxSpeed.times(-driver.getLeftX()))
                                .withRotationalRate(DrivetrainConstants.MaxAngularRate
                                                .times(driver.getRightX()));
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));
        }
}