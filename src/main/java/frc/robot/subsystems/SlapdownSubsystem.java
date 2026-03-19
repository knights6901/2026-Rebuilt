package frc.robot.subsystems;

import static frc.robot.Constants.SlapdownConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem controlling the slapdown mechanism, a hinged arm that deploys
 * to a fixed intake position and retracts to a home position. Driven by a
 * single TalonFX motor with closed-loop position control.
 */
public class SlapdownSubsystem extends SubsystemBase {
    private final TalonFX m_motorSlapdown = new TalonFX(SlapdownMotorId, new CANBus("rio"));
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    /** Tracks whether the slapdown is currently deployed. */
    public boolean isSlapdownDeployed = false;

    private final BooleanPublisher slapdownPub = NetworkTableInstance.getDefault()
            .getTable("Slapdown")
            .getBooleanTopic("SlapdownDeployed")
            .publish();

    private final DoublePublisher slapdownPositionPub = NetworkTableInstance.getDefault()
            .getTable("Slapdown")
            .getDoubleTopic("SlapdownPosition")
            .publish();

    /**
     * Initializes the slapdown subsystem with motor configuration and PID settings.
     * Resets the motor position to home.
     */
    public SlapdownSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = Gains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorSlapdown.getConfigurator().apply(m_motorConfig);
        resetSlapdownPosition();
    }

    /** Moves the slapdown arm to the deployed intake position. */
    public void slapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(IntakePosition));
        isSlapdownDeployed = true;
    }

    /** Retracts the slapdown arm to the stowed home position. */
    public void retractSlapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(HomePosition));
        isSlapdownDeployed = false;
    }

    /**
     * Sets the slapdown motor to a specified duty cycle for manual control.
     *
     * @param power the duty cycle output from -1.0 to 1.0
     */
    public void setPower(double power) {
        m_motorSlapdown.setControl(new DutyCycleOut(power));
    }

    /** Stops the slapdown motor by applying neutral output. */
    public void stop() {
        m_motorSlapdown.setControl(new NeutralOut());
    }

    /**
     * Resets the slapdown motor position encoder to zero (home position).
     */
    public void resetSlapdownPosition() {
        m_motorSlapdown.setPosition(0);
    }

    /**
     * Returns whether the slapdown is currently deployed.
     *
     * @return {@code true} if the slapdown is in the deployed position
     */
    public boolean getDeploymentState() {
        return isSlapdownDeployed;
    }

    @Override
    public void periodic() {
        slapdownPub.set(isSlapdownDeployed);
        slapdownPositionPub.set(m_motorSlapdown.getPosition().getValueAsDouble());
    }
}