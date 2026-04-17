package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

/**
 * Spins up the shooter to a priming speed to prepare it for shooting game
 * pieces, then executes the shooting sequence once the shooter is primed.
 */
public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            LEDSubsystem led,
            Supplier<AngularVelocity> rpsSupplier,
            ShooterState primingState,
            ShooterState shootingState) {
        super(
                new RunCommand(() -> {
                    shooter.shoot(rpsSupplier.get());
                    shooter.shooterState = primingState;
                    led.shooterPattern(() -> shooter.getCurrentRPS(), shooter.getTargetRPS());
                }, shooter, led).until(shooter.primed),
                new ParallelCommandGroup(
                        new RunCommand(() -> {
                            indexer.enable();
                            kicker.kick();
                            shooter.shooterState = shootingState;
                        }, shooter, indexer, kicker),
                        led.runPattern(LEDConstants.RainbowPattern)));
    }
}