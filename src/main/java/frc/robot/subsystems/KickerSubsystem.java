package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.KickerConstants.*;

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX motorKicker = new TalonFX(KickerMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public KickerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = KickerGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorKicker.getConfigurator().apply(m_motorConfig);
    }

    /// Enables the kicker to move balls from the intake to the shooter.
    public void kick() {
        motorKicker.setControl(m_request.withVelocity(KickerRPS));
    }
    
    /// Disables the kicker.
    public void stop() {
        motorKicker.setControl(new NeutralOut());
    }
}