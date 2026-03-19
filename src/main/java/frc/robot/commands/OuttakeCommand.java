package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Reverses the intake mechanism to expel game pieces from the robot.
 * 
 * <p>
 * This command runs continuously until manually interrupted. It starts the
 * outtake process during initialization and allows the intake to eject pieces
 * at the default outtake speed.
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

    /**
     * Initializes the command by starting the intake motor in reverse.
     */
    @Override
    public void initialize() {
        intake.outtake();
    }

    /**
     * This command runs continuously until manually interrupted.
     *
     * @return {@code false} to run continuously
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}