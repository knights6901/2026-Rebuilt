package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motorIntake = new TalonFX(IntakeMotorId, "rio");
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorIntake.getConfigurator().apply(m_motorConfig);
    }

    public void intake() {
        motorIntake.setControl(m_request.withVelocity(IntakeRPS));
    }

    public void outtake() {
        motorIntake.setControl(m_request.withVelocity(IntakeRPS.times(-1.0)));
    }

    public void intake(AngularVelocity rps) {
        motorIntake.setControl(m_request.withVelocity(rps));
    }

    public void outtake(AngularVelocity rps) {
        motorIntake.setControl(m_request.withVelocity(rps.times(-1.0)));
    }

    public void stop() {
        motorIntake.setControl(m_request.withVelocity(0));
    }
}
