package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motorIndexer = new TalonFX(IndexerConstants.IndexerMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private final DoublePublisher indexerVelocityPub = NetworkTableInstance.getDefault()
            .getTable("Indexer")
            .getDoubleTopic("IndexerVelocity")
            .publish();

    public IndexerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IndexerConstants.IndexerGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorIndexer.getConfigurator().apply(m_motorConfig);
    }

    /// Enables the indexer to move balls from the intake to the shooter.
    public void enable() {
        m_motorIndexer.setControl(m_request.withVelocity(IndexerConstants.IndexerRPS));
    }

    /// Disables the indexer.
    public void stop() {
        m_motorIndexer.setControl(m_request.withVelocity(0));
    }

    @Override
    public void periodic() {    
        indexerVelocityPub.set(m_motorIndexer.getVelocity().getValueAsDouble());
    }
}