package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.IndexerConstants.*;

/**
 * Subsystem controlling the indexer mechanism, which transfers game pieces
 * from the intake into the shooter using a single TalonFX motor with
 * closed-loop velocity control.
 */
public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motorIndexer = new TalonFX(MotorId, new CANBus("rio"));
    /**
     * Initializes the indexer subsystem with motor configuration and PID settings.
     */
    public IndexerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(20));

        m_motorIndexer.getConfigurator().apply(m_motorConfig);
    }

    /**
     * Runs the indexer motor at the configured velocity to feed game pieces
     * toward the shooter.
     */
    public void enable() {
        m_motorIndexer.setControl(new DutyCycleOut(Power));
    }

    /**
     * Runs the indexer motor in the reverse direction at the configured velocity.
     */
    public void enableInverted() {
        m_motorIndexer.setControl(new DutyCycleOut(-Power));
    }

    /** Stops the indexer motor by commanding zero velocity. */
    public void stop() {
        m_motorIndexer.setControl(new NeutralOut());
    }
}