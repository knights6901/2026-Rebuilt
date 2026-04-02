package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Shoots at a variable RPM provided by a supplier function, allowing for
 * dynamic control of the shooter's speed based on real-time inputs or
 * adjustments.
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
public class ShootManualRPSCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;

    private final Supplier<AngularVelocity> rpsSupplier;

    /**
     * Constructs a ManualShootCommand with a specific shot RPM.
     *
     * @param shooter  the shooter subsystem
     * @param kicker   the kicker subsystem
     * @param intake   the intake subsystem
     * @param supplier a function providing the RPS to shoot at, at any given
     *                 instance
     */
    public ShootManualRPSCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IndexerSubsystem indexer,
            Supplier<AngularVelocity> rpsSupplier) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.rpsSupplier = rpsSupplier;

        addRequirements(shooter, kicker, indexer);
    }

    @Override
    public void execute() {
        shooter.shoot(rpsSupplier.get());
        indexer.enable();
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
