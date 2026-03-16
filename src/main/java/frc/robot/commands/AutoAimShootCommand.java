package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.IntakeConstants.IndexRPS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoAimShootCommand extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    public AutoAimShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        addRequirements(drivetrain, shooter, kicker, intake);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d hubLocation = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? GameConstants.blueHubLocation
                : GameConstants.redHubLocation;

        Distance shotGroundDistance = Meters
                .of(currentPose.getTranslation().getDistance(hubLocation));

        shooter.shoot(shooter.calculateRPS(shotGroundDistance));
        intake.intake(IndexRPS);
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}