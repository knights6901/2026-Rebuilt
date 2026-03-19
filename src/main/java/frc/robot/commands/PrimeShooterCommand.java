package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Spins up the shooter to a priming speed to prepare it for shooting game pieces.
 *
 * <p>
 * This command spins up the shooter to a priming speed and finishes when the
 * specified timeout has elapsed. Useful for preparing the shooter before a
 * shot.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem}
 */
public class PrimeShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;

    private final Timer timer;
    private final Time timeout;

    /**
     * Constructs a PrimeShooterCommand with a specific timeout.
     *
     * @param shooter the shooter subsystem
     * @param kicker  the kicker subsystem
     * @param timeout the maximum time to run the priming sequence
     */
    public PrimeShooterCommand(ShooterSubsystem shooter, KickerSubsystem kicker, Time timeout) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.timer = new Timer();
        this.timeout = timeout;

        addRequirements(shooter, kicker);
    }

    /**
     * Initializes the command by starting the timer and spinning the shooter
     * at the priming speed.
     */
    @Override
    public void initialize() {
        shooter.shoot(RotationsPerSecond.of(30));
        kicker.kick();
        timer.restart();
    }

    /**
     * Ends the command when the timeout has been reached.
     *
     * @return {@code true} if the timeout has elapsed, {@code false} otherwise
     */
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }
}
