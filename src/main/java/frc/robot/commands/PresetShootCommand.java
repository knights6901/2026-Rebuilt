package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Shoots at a preset RPM configured at command creation time.
 * 
 * <p>
 * This command executes a shooting sequence using a predefined shooter speed
 * (RPM). It coordinates the shooter, kicker, and intake subsystems to perform
 * a complete shot without needing distance calculations.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem},
 * {@link IndexerSubsystem}
 */
public class PresetShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;

    private final AngularVelocity shotRPS;

    /**
     * Constructs a PresetShootCommand with a specific shot RPM.
     *
     * @param shooter the shooter subsystem
     * @param kicker  the kicker subsystem
     * @param intake  the intake subsystem
     * @param shotRPS the preset angular velocity (RPM) for the shot
     */
    public PresetShootCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IndexerSubsystem indexer,
            AngularVelocity shotRPS) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.shotRPS = shotRPS;

        addRequirements(shooter, kicker, indexer);
    }

    @Override
    public void execute() {
        shooter.shoot(shotRPS);
        indexer.enable();
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
