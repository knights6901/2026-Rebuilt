// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static frc.robot.Constants.ShooterConstants.indexRps;
import static frc.robot.Constants.ShooterConstants.shootRps;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SlapdownCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SlapdownSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(ControllerConstants.kDeadband * DrivetrainConstants.MaxSpeed)
                        .withRotationalDeadband(ControllerConstants.kDeadband * DrivetrainConstants.MaxAngularRate) // Add
                                                                                                                    // a
                                                                                                                    // 10%
                                                                                                                    // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
        private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final SlapdownSubsystem slapdown = new SlapdownSubsystem();

        public RobotContainer() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        private void configureDriverBindings() {
                // drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
                // getDriverDrivetrainInput()));
                // drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
                // getDriverInput()));

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driver.getLeftY() * DrivetrainConstants.MaxSpeed) // Drive
                                                                                                                  // forward
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // Y
                                                                                                                  // (forward)
                                                .withVelocityY(-driver.getLeftX() * DrivetrainConstants.MaxSpeed) // Drive
                                                                                                                  // left
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // X
                                                                                                                  // (left)
                                                .withRotationalRate(-driver.getRightX()
                                                                * DrivetrainConstants.MaxAngularRate) // Drive
                                                                                                      // counterclockwise
                                                                                                      // with negative X
                                                                                                      // (left)
                                ));

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
                // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
                                shooter.updateShotVisualization(7, 60);
                        })).onFalse(new InstantCommand(() -> shooter.clearTrajectory()));
                }

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public void configureOperatorBindings() {
                // operator.rightTrigger().whileTrue(new ShooterCommand(shooter,
                // operator.getRightTriggerAxis()));
                // operator.leftTrigger().whileTrue(new IntakeCommand(intake,
                // operator.getLeftTriggerAxis()));
                // operator.a().onTrue(new SlapdownCommand(slapdown));

                // operator.rightTrigger().whileTrue(new InstantCommand(() -> {
                //         shooter.shoot(operator.getRightTriggerAxis());
                // }));

                operator.rightBumper().whileTrue(new RunCommand(() -> {
                        Pose2d currentPose = drivetrain.getState().Pose;
                        double shotGroundDistance = 0;

                        if (DriverStation.getAlliance().isPresent() &&
                                        DriverStation.getAlliance().get() == Alliance.Blue) {
                                shotGroundDistance = gameConstants.blueHubLocation
                                                .getDistance(currentPose.getTranslation());
                        } else if (DriverStation.getAlliance().isPresent() &&
                                        DriverStation.getAlliance().get() == Alliance.Red) {
                                shotGroundDistance = gameConstants.redHubLocation
                                                .getDistance(currentPose.getTranslation());
                        }

                        shooter.shootWithAutoAim(shooter.calculateRPS(ShooterConstants.pitch, shotGroundDistance));
                }));

                operator.rightTrigger().whileFalse(new InstantCommand(() -> {
                        if (!operator.rightBumper().getAsBoolean()) {
                                shooter.stop();
                        }
                }));

                operator.a().whileTrue(new InstantCommand(() -> {
                        shooter.shoot(shootRps);
                }));

                operator.a().whileFalse(new InstantCommand(() -> {
                        shooter.stop();
                }));

                operator.y().whileTrue(new InstantCommand(() -> {
                        shooter.indexer(indexRps);
                }));

                operator.y().whileFalse(new InstantCommand(() -> {
                        shooter.stop();
                }));
        }

        // Generates the command request for moving the drive train based on the current
        // controller input.
        public FieldCentric getDriverInput() {
                double translationX = 0;
                double translationY = 0;
                double angularRotation = 0;

                translationX = -driver.getLeftY() * DrivetrainConstants.MaxSpeed;
                translationY = -driver.getLeftX() * DrivetrainConstants.MaxSpeed;

                angularRotation = driver.getRightX() * DrivetrainConstants.MaxAngularRate;

                return drive.withVelocityX(translationX) // Drive forward with negative Y (forward)
                                .withVelocityY(translationY) // Drive left with negative X (left)
                                .withRotationalRate(angularRotation); // Drive counterclockwise with negative X (left)
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