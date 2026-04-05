package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Automatically aims and shoots toward the hub/goal.
 * 
 * <p>
 * This command calculates the distance from the robot's current position to the
 * target hub location (determined by alliance color), calculates the required
 * shooter
 * RPM based on that distance, and executes the shoot sequence including the
 * indexer
 * and kicker mechanisms.
 * 
 * <p>
 * Requires: {@link CommandSwerveDrivetrain}, {@link ShooterSubsystem},
 * {@link KickerSubsystem}, {@link IndexerSubsystem}
 */
public class ShootAutoRPSCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;

    private final Supplier<Pose2d> currentPoseSupplier;

    /**
     * Constructs an AutoAimShootCommand.
     *
     * @param drivetrain the swerve drivetrain subsystem
     * @param shooter    the shooter subsystem
     * @param kicker     the kicker subsystem
     * @param indexer    the indexer subsystem
     */
    public ShootAutoRPSCommand(
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            Supplier<Pose2d> currentPoseSupplier) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.currentPoseSupplier = currentPoseSupplier;

        addRequirements(shooter, kicker, indexer);
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();
        Translation2d hubLocation = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? GameConstants.BlueHubLocation
                : GameConstants.RedHubLocation;

        Distance shotGroundDistance = Meters
                .of(currentPose.getTranslation().getDistance(hubLocation));

        shooter.shoot(shooter.calculateRPS(shotGroundDistance));

        shooter.shooterState = ShooterSubsystem.ShooterState.AUTOHUB;

        indexer.enable();
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}