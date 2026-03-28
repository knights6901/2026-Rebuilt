// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.BrokenBarrierException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
                        .withDeadband(DrivetrainConstants.MaxSpeed.times(ControllerConstants.Deadband))
                        .withRotationalDeadband(DrivetrainConstants.MaxAngularRate.times(ControllerConstants.Deadband))
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(ControllerConstants.DriverPort);
        private final CommandXboxController operator = new CommandXboxController(ControllerConstants.OperatorPort);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final IndexerSubsystem indexer = new IndexerSubsystem();
        private final SlapdownSubsystem slapdown = new SlapdownSubsystem();
        private final KickerSubsystem kicker = new KickerSubsystem();

        private final SendableChooser<Command> manor_autoChooser;

        boolean isCompetition = false;

        public RobotContainer() {
                configureDriverBindings();
                configureOperatorBindings();

                configureDefaultCommands();
                configurePathPlannerCommands();

                manor_autoChooser = AutoBuilder.buildAutoChooser("zero");

                // mirrored autos for left/right side
                manor_autoChooser.addOption("manor_rightHS", new PathPlannerAuto("manor_leftHS", true));

                SmartDashboard.putData("Auto Chooser", manor_autoChooser);

        }

        /** Registers named commands used by PathPlanner autonomous routines. */
        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("stopSubsystems",
                                new StopSubsystemsCommand(shooter, kicker, intake, indexer));

                NamedCommands.registerCommand("reverseShoot", new RunCommand(() -> {
                        shooter.shoot(RotationsPerSecond.of(-20));
                        kicker.kick(RotationsPerSecond.of(-20));
                }, shooter, kicker).withTimeout(Seconds.of(1)));
                NamedCommands.registerCommand("autoAimShoot",
                                new AutoAimShootCommand(shooter, kicker, indexer, () -> getEstimatedVisionPose()));
                NamedCommands.registerCommand("shoot20RPS",
                                new ManualShootCommand(shooter, kicker, indexer, () -> RotationsPerSecond.of(50)));
                NamedCommands.registerCommand("primeShooter", new PrimeShooterCommand(shooter, kicker, Seconds.of(3)));

                NamedCommands.registerCommand("intake", new IntakeCommand(intake));
                NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stop(), intake));
                NamedCommands.registerCommand("toggleIntake", new ToggleIntakeCommand(intake));

                NamedCommands.registerCommand("rotateToHub",
                                new RotateToHubCommand(drivetrain, () -> getEstimatedVisionPose()));
                NamedCommands.registerCommand("rotate180", new Rotate180Command(drivetrain, drivetrain::getPose));

                NamedCommands.registerCommand("slapdownTrigger", new ToggleSlapdownCommand(slapdown));
        }

        /** Binds all the default commands. */
        private void configureDefaultCommands() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> getDriverInput()));
                kicker.setDefaultCommand(new RunCommand(() -> kicker.stop(), kicker));
                // indexer.setDefaultCommand(new PeriodicReverseIndexerCommand(indexer));
                indexer.setDefaultCommand(new RunCommand(() -> indexer.stop(), indexer));
                intake.setDefaultCommand(new RunCommand(() -> {
                        if (intake.currentlyIntaking())
                                intake.intake();
                        else
                                intake.stop();
                }, intake));
                shooter.setDefaultCommand(new RunCommand(() -> shooter.stop(), shooter));
        }

        /**
         * Binds driver controller inputs to drivetrain commands including
         * field-centric driving, SysId routines, heading reset, and hub tracking.
         */
        private void configureDriverBindings() {
                final var idle = new SwerveRequest.Idle();

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.a().onTrue(new ToggleIntakeCommand(intake));
                driver.x().whileTrue(new OuttakeCommand(intake));
                driver.y().onTrue(new InstantCommand(() -> vision.reseedPose()));
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
                driver.leftBumper().onTrue(new RotateToHubCommand(
                                drivetrain,
                                () -> getEstimatedVisionPose()));

                if (Robot.isSimulation()) {
                        driver.x().onTrue(new InstantCommand(() -> {
                                shooter.updateShotVisualization(drivetrain.getPose(), 7, 60);
                        })).onFalse(new InstantCommand(() -> shooter.clearTrajectory()));
                }

                driver.povUp().onTrue(new InstantCommand(() -> slapdown.retractSlapdown(), slapdown));
                driver.povDown().onTrue(new InstantCommand(() -> slapdown.slapdown(), slapdown));

                driver.rightStick().onTrue(new RunCommand(() -> slapdown.resetSlapdownPosition(), slapdown));
                driver.leftStick().onTrue(
                                new InstantCommand(() -> drivetrain.applyRequest(() -> getDriverInput()), drivetrain));

                driver.povRight().whileTrue(Commands.startEnd(
                                () -> slapdown.setPower(0.1),
                                () -> slapdown.stop(),
                                slapdown));
                driver.povLeft().whileTrue(Commands.startEnd(
                                () -> slapdown.setPower(-0.1),
                                () -> slapdown.stop(),
                                slapdown));

                driver.leftTrigger().whileTrue(new RunCommand(() -> {
                        indexer.enableInverted();
                        kicker.enableInverted();
                }, indexer));

                driver.rightTrigger().whileTrue(
                                new AutoAimShootCommand(shooter, kicker, indexer, () -> getEstimatedVisionPose()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Binds operator controller inputs to scoring-mechanism commands including
         * shooting, intake, and auto-aim.
         */
        private void configureOperatorBindings() {
                operator.leftBumper().onTrue(new ToggleIntakeCommand(intake));

                operator.povLeft().onTrue(new InstantCommand(() -> {
                        shooter.decreaseShootRPS();
                }));
                operator.povRight().onTrue(new InstantCommand(() -> {
                        shooter.increaseShootRPS();
                }));

                operator.rightTrigger().whileTrue(
                                new ManualShootCommand(shooter, kicker, indexer, () -> shooter.getShootRPS()));

                operator.rightBumper().onTrue(new RunCommand(() -> slapdown.resetSlapdownPosition(), slapdown));

                // operator.rightBumper().whileTrue(
                // new ManualShootCommand(shooter, kicker, indexer, () ->
                // ShooterConstants.DefaultRPS));

                operator.leftTrigger().whileTrue(
                                new AutoAimShootCommand(shooter, kicker, indexer, () -> getEstimatedVisionPose()));

                operator.povUp().onTrue(new PrimeShooterCommand(shooter, kicker, Seconds.of(5)));
                operator.povDown().whileTrue(new StopSubsystemsCommand(shooter, kicker, intake, indexer));

                operator.x().onTrue(new Rotate180Command(drivetrain, () -> drivetrain.getPose()));

                operator.b().whileTrue(new RunCommand(() -> {
                        indexer.enableInverted();
                }, indexer));
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
                                                -driver.getLeftY() * DrivetrainConstants.TeleopMovementSensitivity))
                                .withVelocityY(DrivetrainConstants.MaxSpeed.times(
                                                -driver.getLeftX() * DrivetrainConstants.TeleopMovementSensitivity))
                                .withRotationalRate(DrivetrainConstants.MaxAngularRate
                                                .times(-driver.getRightX()));
        }

        /**
         * Returns the autonomous command selected from the SmartDashboard chooser.
         *
         * @return the selected autonomous {@link Command}
         */
        public Command getAutonomousCommand() {
                return manor_autoChooser.getSelected();
        }

        /**
         * Gets the robot's estimated pose from the vision subsystem, or falls back to
         * the drivetrain's odometry pose if no vision estimate is available.
         *
         * @return the estimated {@link Pose2d} of the robot on the field
         */
        private Pose2d getEstimatedVisionPose() {
                return vision.getEstimatedPose2d().orElse(drivetrain.getState().Pose);
        }
}