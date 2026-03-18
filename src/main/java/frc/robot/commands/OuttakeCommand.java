package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Reverses the intake mechanism to expel game pieces from the robot.
 * 
 * <p>
 * This command runs continuously until manually interrupted. It simply calls
 * the
 * intake subsystem's outtake method during initialization.
 * 
 * <p>
 * Requires: {@link IntakeSubsystem}
 */
public class OuttakeCommand extends Command {
    private final IntakeSubsystem intake;

    /**
     * Constructs an OuttakeCommand.
     *
     * @param intake the intake subsystem
     */
    public OuttakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.outtake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}