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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * Declares all subsystems, operator-interface devices, and command bindings.
 *
 * <p>
 * Instantiates the drivetrain, vision, shooter, intake, indexer, slapdown,
 * and kicker subsystems, wires them to Xbox controller inputs for both the
 * driver and operator, registers PathPlanner named commands for autonomous
 * routines, and exposes the autonomous chooser.
 */
public class RobotContainer {
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConstants.MaxSpeed.times(ControllerConstants.kDeadband))
                        .withRotationalDeadband(DrivetrainConstants.MaxAngularRate.times(ControllerConstants.kDeadband))
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
        private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final IndexerSubsystem indexer = new IndexerSubsystem();
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

        /** Registers named commands used by PathPlanner autonomous routines. */
        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("stopSubsystems",
                                new StopSubsystemsCommand(shooter, kicker, intake, indexer));

                NamedCommands.registerCommand("autoAimShoot",
                                new AutoAimShootCommand(drivetrain, shooter, kicker, indexer)
                                                .withTimeout(Seconds.of(3.0)));
                NamedCommands.registerCommand("shoot20RPS",
                                new PresetShootCommand(shooter, kicker, indexer, RotationsPerSecond.of(20)));

                NamedCommands.registerCommand("intake", new IntakeCommand(intake));
                NamedCommands.registerCommand("rotateToHub", new RotateToHubCommand(drivetrain));
                NamedCommands.registerCommand("slapdownTrigger", new TriggerSlapdownCommand(slapdown));
        }

        /**
         * Binds driver controller inputs to drivetrain commands including
         * field-centric driving, SysId routines, heading reset, and hub tracking.
         */
        private void configureDriverBindings() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> getDriverInput()));
                slapdown.setDefaultCommand(new RunCommand(() -> slapdown.stop(), slapdown));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.a().whileTrue(new IntakeCommand(intake));
                driver.povDown().whileTrue(new OuttakeCommand(intake));

                driver.b().whileTrue(drivetrain.applyRequest(() -> brake));

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

                // TEMPORARY
                // Slowly move slapdown down
                driver.povLeft().whileTrue(
                                new RunCommand(() -> slapdown.setSpeed(RotationsPerSecond.of(1)), slapdown));
                driver.povRight().whileTrue(
                                new RunCommand(() -> slapdown.setSpeed(RotationsPerSecond.of(-1)),
                                                slapdown));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Binds operator controller inputs to scoring-mechanism commands including
         * shooting, intake, and auto-aim.
         */
        private void configureOperatorBindings() {
                kicker.setDefaultCommand(new RunCommand(() -> kicker.stop(), kicker));
                indexer.setDefaultCommand(new RunCommand(() -> indexer.stop(), indexer));
                intake.setDefaultCommand(new RunCommand(() -> intake.stop(), intake));
                shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));

                operator.leftTrigger().whileTrue(
                                new AutoAimShootCommand(drivetrain, shooter, kicker, indexer));

                operator.rightBumper().whileTrue(
                                new PresetShootCommand(shooter, kicker, indexer,
                                                ShooterConstants.MaxRPS.times(operator.getRightY())));

                operator.rightBumper().whileTrue(
                                new PresetShootCommand(shooter, kicker, indexer, ShooterConstants.ShootRPS));

                operator.povUp().onTrue(new PrimeShooterCommand(shooter, Seconds.of(5)));
                operator.povDown().whileTrue(new StopSubsystemsCommand(shooter, kicker, intake, indexer));
        }

        /**
         * Builds a field-centric drive request from the driver controller's joystick
         * axes.
         *
         * @return the {@link FieldCentric} request with velocity and rotation applied
         */
        public FieldCentric getDriverInput() {
                return drive
                                .withVelocityX(DrivetrainConstants.MaxSpeed.times(
                                                driver.getLeftY() * DrivetrainConstants.TeleopMovementSensitivity))
                                .withVelocityY(DrivetrainConstants.MaxSpeed.times(
                                                driver.getLeftX() * DrivetrainConstants.TeleopMovementSensitivity))
                                .withRotationalRate(DrivetrainConstants.MaxAngularRate
                                                .times(-driver.getRightX()));
        }

        /**
         * Returns the autonomous command selected from the SmartDashboard chooser.
         *
         * @return the selected autonomous {@link Command}
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}