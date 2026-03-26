package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

import static frc.robot.Constants.PeriodicReverseIndexerConstants.*;

public class PeriodicReverseIndexerCommand extends Command {
    private final IndexerSubsystem indexer;

    private final Timer lastReverse;
    private final Timer reverseCooldown;

    public PeriodicReverseIndexerCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;

        this.lastReverse = new Timer();
        this.reverseCooldown = new Timer();

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        this.lastReverse.start();
    }

    @Override
    public void execute() {
        if (this.reverseCooldown.hasElapsed(ReverseLength)) {
            this.indexer.stop();
        }

        if (this.lastReverse.hasElapsed(ReversePeriod)) {
            this.lastReverse.restart();
            this.reverseCooldown.restart();

            this.indexer.enableInverted();
        } else {
            this.indexer.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
