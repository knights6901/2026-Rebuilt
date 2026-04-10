package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.SlapdownConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownConstants;

/**
 * Subsystem controlling the slapdown mechanism, a hinged arm that deploys
 * to a fixed intake position and retracts to a home position. Driven by a
 * single TalonFX motor with closed-loop position control.
 */
public class SlapdownSubsystem extends SubsystemBase {
    /** The possible states of the slapdown mechanism. */
    public static enum SlapdownState {
        UP,
        DOWN
    }

    private final TalonFX m_motorSlapdown = new TalonFX(MotorId, new CANBus("rio"));
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    /** The current state of the slapdown mechanism. */
    public SlapdownState state = SlapdownState.UP;

    /**
     * 
     * Initializes the slapdown subsystem with motor configuration and PID settings.
     * Resets the motor position to home.
     */
    public SlapdownSubsystem() {
        m_motorSlapdown.getConfigurator().apply(SlapdownConstants.MotorConfig);
        resetSlapdownPosition();
    }

    /** Moves the slapdown arm to the deployed intake position. */
    public void slapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(IntakePosition));
        state = SlapdownState.DOWN;
    }

    /** Retracts the slapdown arm to the stowed home position. */
    public void retractSlapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(HomePosition));
        state = SlapdownState.UP;
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
        state = SlapdownState.UP;
    }

    /**
     * Returns whether the slapdown is currently deployed.
     *
     * @return
     */
    public SlapdownState getDeploymentState() {
        Angle error = (state == SlapdownState.DOWN ? IntakePosition : HomePosition).minus(getSlapdownPosition());

        if (error.abs(Degrees) / 144 <= PositionTolerance.in(Degrees)) {
            return state;
        } else {
            return state == SlapdownState.DOWN ? SlapdownState.UP : SlapdownState.DOWN;
        }
    }

    /* Returns the current position of the slapdown motor. */
    public Angle getSlapdownPosition() {
        return m_motorSlapdown.getPosition().getValue();
    }
}