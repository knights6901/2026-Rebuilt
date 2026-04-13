package frc.robot.commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPassRPSCommand extends Command{
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;

    private final Supplier<Pose2d> currentPoseSupplier;

    public ShootPassRPSCommand(
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            Supplier<Pose2d> currentPoseSupplier) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.currentPoseSupplier = currentPoseSupplier;

        addRequirements(shooter, kicker, indexer);
    }

    @Override
    public void execute() {
        Pose2d currentPose = currentPoseSupplier.get();
        Distance targetX = (DriverStation.getAlliance().get() == Alliance.Blue)
                ? Meters.of(2.306)
                : Meters.of(14.207);

        Distance shotGroundDistance = Meters
                .of(Math.abs(currentPose.getTranslation().getX() - targetX.in(Meters)));
        Distance vertDistance = Meters.of(0);

        shooter.shoot(shooter.calculateRPS(shotGroundDistance, vertDistance));
        shooter.shooterState = ShooterSubsystem.ShooterState.AUTOPASS;

        if (shooter.isPrimed()) {
            indexer.enable();
            kicker.kick();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
