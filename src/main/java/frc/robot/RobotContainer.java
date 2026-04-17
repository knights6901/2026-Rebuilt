// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StopSubsystemsCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SlapdownSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
        public final LEDSubsystem led = new LEDSubsystem(drivetrain);

        private final SendableChooser<Command> dcmp_autoChooser;

        boolean isCompetition = false;

        public RobotContainer() {
                configureDriverBindings();
                configureOperatorBindings();

                configureDefaultCommands();
                configurePathPlannerCommands();

                dcmp_autoChooser = AutoBuilder.buildAutoChooser("zero");

                // mirrored left autos for right side
                dcmp_autoChooser.addOption("dcmp_rightHS", new PathPlannerAuto("dcmp_leftHS", true));
                dcmp_autoChooser.addOption("dcmp_rightHS_delay", new PathPlannerAuto("dcmp_leftHS_delay", true));

                dcmp_autoChooser.addOption("dcmp_rightChaos", new PathPlannerAuto("dcmp_leftChaos", true));
                dcmp_autoChooser.addOption("dcmp_rightChaos_delay", new PathPlannerAuto("dcmp_leftChaos_delay", true));

                dcmp_autoChooser.addOption("dcmp_rightPass", new PathPlannerAuto("dcmp_leftPass", true));
                dcmp_autoChooser.addOption("dcmp_rightPass_delay", new PathPlannerAuto("dcmp_leftPass_delay", true));

                // mirrored left autos for right side, but they're AP's sketchy ideas
                dcmp_autoChooser.addOption("dcmp_rightDoubleHS", new PathPlannerAuto("dcmp_leftDoubleHS", true));
                dcmp_autoChooser.addOption("dcmp_rightHS_disrupt", new PathPlannerAuto("dcmp_leftHS_disrupt", true));
                dcmp_autoChooser.addOption("dcmp_rightHS_disruptBump",
                                new PathPlannerAuto("dcmp_leftHS_disruptBump", true));
                dcmp_autoChooser.addOption("dcmp_rightHS_returnBump",
                                new PathPlannerAuto("dcmp_leftHS_returnBump", true));

                dcmp_autoChooser.addOption("dcmp_right_ap_sketchy_auton",
                                new PathPlannerAuto("dcmp_left_ap_sketchy_auton", true));

                SmartDashboard.putData("Auto Chooser", dcmp_autoChooser);
        }

        /** Registers named commands used by PathPlanner autonomous routines. */
        private void configurePathPlannerCommands() {
                NamedCommands.registerCommand("stopSubsystems",
                                new StopSubsystemsCommand(shooter, kicker, intake, indexer));

                NamedCommands.registerCommand("autoAimShoot",
                                shooter.autoAimShoot(this::getEstimatedVisionPose, kicker, indexer, led));

                NamedCommands.registerCommand("autoPassShoot",
                                shooter.passShoot(this::getEstimatedVisionPose, kicker, indexer, led));

                NamedCommands.registerCommand("fiftyRPSShoot",
                                shooter.manuallyShoot(() -> RotationsPerSecond.of(50), kicker, indexer, led));

                NamedCommands.registerCommand("primeShooter", shooter.prime().withTimeout(Seconds.of(3)));

                NamedCommands.registerCommand("intake", new IntakeCommand(intake));
                NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stop(), intake));
                NamedCommands.registerCommand("toggleIntake", new ToggleIntakeCommand(intake));

                NamedCommands.registerCommand("rotateToHub",
                                drivetrain.rotateToHub(this::nullDriverInput));
                NamedCommands.registerCommand("rotate180",
                                drivetrain.rotateBy180(this::nullDriverInput));

                NamedCommands.registerCommand("slapdownTrigger", slapdown.slapdownCommand());
                NamedCommands.registerCommand("slapdownRetract", slapdown.retractSlapdownCommand());
        }

        /** Binds all the default commands. */
        private void configureDefaultCommands() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> getDriverInput()));
                kicker.setDefaultCommand(new RunCommand(() -> kicker.stop(), kicker));
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
                driver.x().onTrue(shooter.prime());
                driver.y().onTrue(new InstantCommand(() -> vision.reseedPose()));
                driver.b().whileTrue(drivetrain.applyRequest(() -> brake));
                driver.leftStick().onTrue(
                                new InstantCommand(() -> drivetrain.applyRequest(() -> getDriverInput()), drivetrain));

                driver.y().whileTrue(new RunCommand(() -> indexer.enable(), indexer));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on right bumper press.
                driver.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                driver.leftBumper().onTrue(drivetrain.rotateToHub(this::getDriverInput));

                driver.povUp().onTrue(slapdown.retractSlapdownCommand());
                driver.povDown().onTrue(slapdown.slapdownCommand());

                driver.rightStick().onTrue(new RunCommand(() -> slapdown.resetSlapdownPosition(), slapdown));

                driver.povLeft().whileTrue(Commands.startEnd(
                                () -> slapdown.setPower(0.36901),
                                () -> slapdown.stop(),
                                slapdown));
                driver.povRight().whileTrue(Commands.startEnd(
                                () -> slapdown.setPower(-0.36901),
                                () -> slapdown.stop(),
                                slapdown));

                driver.leftTrigger().whileTrue(
                                shooter.passShoot(this::getEstimatedVisionPose, kicker, indexer, led));
                driver.rightTrigger().whileTrue(
                                shooter.autoAimShoot(this::getEstimatedVisionPose, kicker, indexer, led));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Binds operator controller inputs to scoring-mechanism commands including
         * shooting, intake, and auto-aim.
         */
        private void configureOperatorBindings() {
                led.runPattern(LEDConstants.RainbowPattern);

                operator.leftBumper().onTrue(new ToggleIntakeCommand(intake));

                operator.povLeft().onTrue(new InstantCommand(() -> {
                        shooter.decreaseShootRPS();
                }));
                operator.povRight().onTrue(new InstantCommand(() -> {
                        shooter.increaseShootRPS();
                }));

                operator.rightTrigger().whileTrue(
                                shooter.manuallyShoot(() -> shooter.getShootRPS(), kicker, indexer, led));

                operator.rightBumper().onTrue(new RunCommand(() -> slapdown.resetSlapdownPosition(), slapdown));

                operator.leftTrigger().whileTrue(
                                shooter.autoAimShoot(this::getEstimatedVisionPose, kicker, indexer, led));

                operator.povUp().onTrue(shooter.prime());
                operator.povDown().whileTrue(new StopSubsystemsCommand(shooter, kicker, intake, indexer));

                operator.x().onTrue(drivetrain.rotateBy180(this::nullDriverInput));
                operator.y().onTrue(drivetrain.alignToTrench(this::getDriverInput));
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

        /** Returns a driver input with all values set to zero. */
        public FieldCentric nullDriverInput() {
                return drive
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0);
        }

        /**
         * Returns the autonomous command selected from the SmartDashboard chooser.
         *
         * @return the selected autonomous {@link Command}
         */
        public Command getAutonomousCommand() {
                return dcmp_autoChooser.getSelected();
        }

        /**
         * Gets the robot's estimated pose from the vision subsystem, or falls back to
         * the drivetrain's odometry pose if no vision estimate is available.
         *
         * @return the estimated {@link Pose2d} of the robot on the field
         */
        private Pose2d getEstimatedVisionPose() {
                Translation2d visionTranslation = vision.getEstimatedPose2d().orElse(drivetrain.getState().Pose)
                                .getTranslation();
                Rotation2d drivetrainRotation = drivetrain.getPose().getRotation();

                return new Pose2d(visionTranslation, drivetrainRotation);
                // return vision.getEstimatedPose2d().orElse(drivetrain.getPose());
        }
}