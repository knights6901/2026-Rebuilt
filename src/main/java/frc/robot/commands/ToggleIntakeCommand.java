package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Activates the intake mechanism to draw game pieces into the robot.
 * 
 * <p>
 * This command toggles the intake between running and stopped states. If the
 * intake is already running, this command stops it; if stopped, it starts
 * intaking.
 * 
 * <p>
 * Requires: {@link IntakeSubsystem}
 */
public class ToggleIntakeCommand extends Command {
    private final IntakeSubsystem intake;

    /**
     * Constructs a ToggleIntakeCommand.
     *
     * @param intake the intake subsystem
     */
    public ToggleIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    /**
     * Initializes the command by toggling the intake state.
     * If currently intaking, it stops; otherwise it starts intaking.
     */
    @Override
    public void initialize() {
        if (intake.intaking())
            intake.stop();
        else
            intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}