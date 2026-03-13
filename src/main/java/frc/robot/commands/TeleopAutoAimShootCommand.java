package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopAutoAimShootCommand extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private ShooterSubsystem shooter;

    public TeleopAutoAimShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d hubLocation = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? GameConstants.blueHubLocation
                : GameConstants.redHubLocation;

        Distance shotGroundDistance = Meters
                .of(currentPose.getTranslation().getDistance(hubLocation));

        shooter.shootWithAutoAim(shooter.calculateRPS(shotGroundDistance));
    }
}
