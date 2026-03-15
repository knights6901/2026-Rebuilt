package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonIntakeCommand extends Command {
    private final IntakeSubsystem intake;

    public AutonIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake(IntakeConstants.IndexRPS);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}