package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

/**
 * Enables the shooter to run at a low RPS to make it ready to actually shoot
 * fuel.
 * <p>
 * Requires: {@link ShooterSubsystem}
 */
public class PrimeShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    private final Timer timer;
    private final Time timeout;

    /**
     * Constructs a PrimeShooterCommand with a specific timeout.
     *
     * @param shooter the shooter subsystem
     */
    public PrimeShooterCommand(ShooterSubsystem shooter, Time timeout) {
        this.shooter = shooter;
        this.timer = new Timer();
        this.timeout = timeout;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot(RotationsPerSecond.of(30));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }
}
