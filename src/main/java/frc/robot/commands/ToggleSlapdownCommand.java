package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SlapdownSubsystem;

/**
 * Toggles the slapdown mechanism deployment state.
 * 
 * <p>
 * This command checks the current deployment state of the slapdown subsystem
 * and
 * toggles it - if deployed, it retracts; if retracted, it deploys. The command
 * finishes
 * when the deployment state changes from its initial state.
 * 
 * <p>
 * Requires: {@link SlapdownSubsystem}
 */
public class ToggleSlapdownCommand extends Command {
    private final SlapdownSubsystem slapdown;
    private boolean initialState;

    /**
     * Constructs a TriggerSlapdownCommand.
     *
     * @param slapdown the slapdown subsystem
     */
    public ToggleSlapdownCommand(SlapdownSubsystem slapdown) {
        this.slapdown = slapdown;
        this.initialState = slapdown.getDeploymentState();

        addRequirements(slapdown);
    }

    @Override
    public void execute() {
        if (!slapdown.isSlapdownDeployed) {
            // slapdown.slapdown();
        } else {
            // slapdown.retractSlapdown();
        }
    }

    @Override
    public boolean isFinished() {
        // return initialState != slapdown.getDeploymentState();
        return true;
    }
}
