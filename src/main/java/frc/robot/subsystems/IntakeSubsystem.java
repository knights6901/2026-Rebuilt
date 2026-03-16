package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motorIntake = new TalonFX(IntakeMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorIntake.getConfigurator().apply(m_motorConfig);
    }

    public void intake() {
        m_motorIntake.setControl(m_request.withVelocity(IntakeRPS));
    }

    public void outtake() {
        m_motorIntake.setControl(m_request.withVelocity(IntakeRPS.times(-1.0)));
    }

    public void intake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps));
    }

    public void outtake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps.times(-1.0)));
    }

    public void stop() {
        m_motorIntake.setControl(new NeutralOut());
    }
}
