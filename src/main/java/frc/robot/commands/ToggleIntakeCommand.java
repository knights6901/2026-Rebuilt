package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Activates the intake mechanism to draw game pieces into the robot.
 * 
 * <p>
 * This command runs continuously until manually interrupted. It simply calls
 * the
 * intake subsystem's intake method during initialization.
 * 
 * <p>
 * Requires: {@link IntakeSubsystem}
 */
public class ToggleIntakeCommand extends Command {
    private final IntakeSubsystem intake;

    /**
     * Constructs an IntakeCommand.
     *
     * @param intake the intake subsystem
     */
    public ToggleIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.intaking())
            intake.stop();
        else
            intake.intake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}