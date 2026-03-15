package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Auton20RPSShootCommand extends Command {
    private ShooterSubsystem shooter;

    public Auton20RPSShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(20);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
