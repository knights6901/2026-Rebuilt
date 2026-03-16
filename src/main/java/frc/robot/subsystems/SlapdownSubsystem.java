package frc.robot.subsystems;

import static frc.robot.Constants.SlapdownConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlapdownSubsystem extends SubsystemBase {
    private final TalonFX m_motorSlapdown = new TalonFX(SlapdownMotorId, "rio");
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public boolean isSlapdownDeployed = false;

    public SlapdownSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = SlapdownGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorSlapdown.getConfigurator().apply(m_motorConfig);
        m_motorSlapdown.setPosition(0);
    }

    /// Deploys the slapdown.
    public void slapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(IntakePosition));
        isSlapdownDeployed = true;
    }

    /// Retracts the slapdown.
    public void retractSlapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(HomePosition));
        isSlapdownDeployed = false;
    }

    public boolean getDeploymentState() {
        return isSlapdownDeployed;
    }
}
