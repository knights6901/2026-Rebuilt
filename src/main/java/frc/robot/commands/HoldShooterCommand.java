// will stop every subsystem except shooter

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class HoldShooterCommand extends Command {
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    public HoldShooterCommand(KickerSubsystem kicker, IntakeSubsystem intake) {
        this.kicker = kicker;
        this.intake = intake;
        addRequirements(kicker, intake);
    }

    @Override
    public void execute() {
        kicker.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}