// if you need to stop literally any subsytem, just use this
// if shooter stops, everything will have to stop anways
// if you need something other than shooter to stop while shooter runs, use holdshootercommand instead

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopSubsystemsCommand extends Command {
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    public StopSubsystemsCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        addRequirements(shooter, kicker,intake);
    }

    @Override
    public void execute() {
        shooter.stop();
        kicker.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}