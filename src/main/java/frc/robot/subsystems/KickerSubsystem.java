package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.KickerConstants;

/**
 * Subsystem controlling the kicker wheel, which provides the final push
 * to transfer game pieces from the indexer into the shooter flywheel.
 * Uses a single TalonFX motor with closed-loop velocity control.
 */
public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_motorKicker = new TalonFX(KickerConstants.MotorId, new CANBus("rio"));
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final DoublePublisher kickerPub = NetworkTableInstance.getDefault()
            .getTable("Kicker")
            .getDoubleTopic("KickerVelocity")
            .publish();

    /**
     * Initializes the kicker subsystem with motor configuration, PID settings,
     * and current limiting.
     */
    public KickerSubsystem() {
        m_motorKicker.getConfigurator().apply(KickerConstants.MotorConfig);
    }

    public void kick(AngularVelocity rps) {
        m_motorKicker.setControl(m_request.withVelocity(rps));
    }

    /**
     * Spins the kicker wheel at the configured velocity to feed a game piece into
     * the shooter.
     */
    public void kick() {
        kick(KickerConstants.KickerRPS);
    }

    public void enableInverted() {
        kick(KickerConstants.KickerRPS.times(-0.25));
    }

    /** Stops the kicker motor by applying neutral output. */
    public void stop() {
        m_motorKicker.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {
        kickerPub.set(m_motorKicker.getVelocity().getValueAsDouble());
    }
}