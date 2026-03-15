package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SlapdownSubsystem;

public class TriggerSlapdownCommand extends Command {
    private final SlapdownSubsystem slapdown;

    public TriggerSlapdownCommand(SlapdownSubsystem slapdown) {
        this.slapdown = slapdown;

        addRequirements(slapdown);
    }

    @Override
    public void execute() {
        if (!slapdown.isSlapdownDeployed) {
            slapdown.slapdown();
        } else {
            slapdown.retractSlapdown();
        }
    }
    
}
