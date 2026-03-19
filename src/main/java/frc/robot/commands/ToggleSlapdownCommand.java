package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SlapdownSubsystem;
import frc.robot.subsystems.SlapdownSubsystem.SlapdownState;

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
    private SlapdownState initialState;

    /**
     * Constructs a ToggleSlapdownCommand.
     *
     * @param slapdown the slapdown subsystem
     */
    public ToggleSlapdownCommand(SlapdownSubsystem slapdown) {
        this.slapdown = slapdown;
        this.initialState = slapdown.getDeploymentState();

        addRequirements(slapdown);
    }

    /**
     * Toggles the slapdown deployment state.
     */
    @Override
    public void initialize() {
        if (initialState == SlapdownState.UP) {
            slapdown.slapdown();
        } else {
            slapdown.retractSlapdown();
        }
    }

    /**
     * Finishes the command when the deployment state changes.
     *
     * @return {@code true} once the deployment state has changed
     */
    @Override
    public boolean isFinished() {
        return initialState != slapdown.getDeploymentState();
    }
}
