// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;

import frc.robot.commands.*;
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
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final SlapdownSubsystem slapdown = new SlapdownSubsystem();
        private final KickerSubsystem kicker = new KickerSubsystem();

        private final SendableChooser<Command> autoChooser;

        boolean isCompetition = false;

        public RobotContainer() {
                configureDriverBindings();
                configureOperatorBindings();

                configurePathPlannerCommands();

                autoChooser = AutoBuilder.buildAutoChooser("zero");

                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("stopSubsystems", new StopSubsystemsCommand(shooter, kicker, intake));

                NamedCommands.registerCommand("autoAimShoot",
                        new AutoAimShootCommand(drivetrain, shooter, kicker, intake).withTimeout(Seconds.of(3.0)));
                NamedCommands.registerCommand("holdShooter", new HoldShooterCommand(kicker, intake));
                NamedCommands.registerCommand("shoot20RPS", new PresetShootCommand(shooter, kicker, intake, RotationsPerSecond.of(20)));
                
                NamedCommands.registerCommand("intake", new IntakeCommand(intake));
                NamedCommands.registerCommand("rotateToHub", new RotateToHubCommand(drivetrain));
                NamedCommands.registerCommand("slapdownTrigger", new TriggerSlapdownCommand(slapdown));
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

                // Reset the field-centric heading on right bumper press.
                driver.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Point the wheels towards the hub when holding left bumper.
                driver.leftBumper().whileTrue(new RotateToHubCommand(drivetrain));

                if (Robot.isSimulation()) {
                        driver.x().onTrue(new InstantCommand(() -> {
                                shooter.updateShotVisualization(drivetrain.getPose(), 7, 60);
                        })).onFalse(new InstantCommand(() -> shooter.clearTrajectory()));
                }

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureOperatorBindings() {
                shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));
                kicker.setDefaultCommand(new RunCommand(() -> kicker.stop(), kicker));
                intake.setDefaultCommand(new RunCommand(() -> intake.stop(), intake));

                operator.leftBumper().whileTrue(new AutoAimShootCommand(drivetrain, shooter, kicker, intake));

                operator.a().whileTrue(new IntakeCommand(intake));

                operator.rightBumper().whileTrue(new PresetShootCommand(shooter, kicker, intake, ShooterConstants.ShootRPS));

                operator.rightTrigger().whileTrue(new InstantCommand(() -> {
                        shooter.shoot(operator.getRightTriggerAxis() * ShooterConstants.maxRPS);
                }));

                operator.b().whileTrue(new InstantCommand(() -> {
                        kicker.kick();
                }));
        }

        // Generates the command request for moving the drive train based on the current
        // controller input.
        public FieldCentric getDriverInput() {
                return drive
                                .withVelocityX(DrivetrainConstants.MaxSpeed.times(driver.getLeftY()))
                                .withVelocityY(DrivetrainConstants.MaxSpeed.times(driver.getLeftX()))
                                .withRotationalRate(DrivetrainConstants.MaxAngularRate
                                                .times(-driver.getRightX()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}