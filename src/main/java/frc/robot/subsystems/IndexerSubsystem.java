package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Subsystem controlling the indexer mechanism, which transfers game pieces
 * from the intake into the shooter using a single TalonFX motor with
 * closed-loop velocity control.
 */
public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motorIndexer = new TalonFX(IndexerConstants.MotorId, new CANBus("rio"));

    private final DoublePublisher indexerVelocityPub = NetworkTableInstance.getDefault()
            .getTable("Indexer")
            .getDoubleTopic("IndexerVelocity")
            .publish();

    public IndexerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorIndexer.getConfigurator().apply(m_motorConfig);
    }

    /**
     * Runs the indexer motor at the configured velocity to feed game pieces
     * toward the shooter.
     */
    public void enable() {
        m_motorIndexer.setControl(new DutyCycleOut(IndexerConstants.Power));
    }

    public void enableInverted() {
        m_motorIndexer.setControl(new DutyCycleOut(-IndexerConstants.Power));
    }

    /** Stops the indexer motor by commanding zero velocity. */
    public void stop() {
        m_motorIndexer.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {
        indexerVelocityPub.set(m_motorIndexer.getVelocity().getValueAsDouble());
    }
}