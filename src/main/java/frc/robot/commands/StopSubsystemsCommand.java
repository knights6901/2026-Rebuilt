// if you need to stop literally any subsytem, just use this
// if shooter stops, everything will have to stop anways
// if you need something other than shooter to stop while shooter runs, use holdshootercommand instead

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Stops all shooter-related subsystems.
 * 
 * <p>
 * This command disables the shooter, kicker, and intake subsystems. Use this
 * command
 * to halt all shooting-related mechanisms at once. If you need more
 * fine-grained control
 * where some subsystems continue while others stop, consider using alternative
 * commands
 * with more specific requirements.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem},
 * {@link IntakeSubsystem}, {@link IndexerSubsystem}
 */
public class StopSubsystemsCommand extends Command {
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;

    /**
     * Constructs a StopSubsystemsCommand.
     *
     * @param shooter the shooter subsystem to stop
     * @param kicker  the kicker subsystem to stop
     * @param intake  the intake subsystem to stop
     * @param indexer the indexer subsystem to stop
     */
    public StopSubsystemsCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(shooter, kicker, intake, indexer);
    }

    @Override
    public void execute() {
        shooter.stop();
        kicker.stop();
        intake.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}