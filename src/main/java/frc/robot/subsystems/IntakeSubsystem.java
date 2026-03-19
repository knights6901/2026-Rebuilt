package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem controlling the intake rollers, used to pull game pieces into
 * the robot or eject them. Driven by a single TalonFX motor with closed-loop
 * velocity control.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motorIntake = new TalonFX(IntakeMotorId, new CANBus("rio"));
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private boolean intaking = false;

    private final DoublePublisher intakeVelocityPub = NetworkTableInstance.getDefault()
            .getTable("Intake")
            .getDoubleTopic("IntakeVelocity")
            .publish();
    private final BooleanPublisher intakingPub = NetworkTableInstance.getDefault()
            .getTable("Intake")
            .getBooleanTopic("IntakeToggled")
            .publish();

    /**
     * Initializes the intake subsystem with motor configuration and PID settings.
     */
    public IntakeSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = Gains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motorIntake.getConfigurator().apply(m_motorConfig);
    }

    /** Runs the intake rollers inward at the default velocity. */
    public void intake() {
        intake(IntakeRPS);
    }

    /**
     * Runs the intake rollers inward at a custom velocity.
     *
     * @param rps target angular velocity for the intake motor
     */
    public void intake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps));
        intaking = true;
    }

    /**
     * Runs the intake rollers outward at the default velocity to eject game pieces.
     */
    public void outtake() {
        outtake(IntakeRPS);
    }

    /**
     * Runs the intake rollers outward at a custom velocity to eject game pieces.
     *
     * @param rps target angular velocity magnitude (will be negated internally)
     */
    public void outtake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps.times(-1.0)));
        intaking = false;
    }

    /** Stops the intake motor by applying neutral output. */
    public void stop() {
        m_motorIntake.setControl(new NeutralOut());
        intaking = false;
    }

    /**
     * Gets the current intake state.
     *
     * @return {@code true} if the intake is actively intaking, {@code false}
     *         otherwise
     */
    public boolean intaking() {
        return intaking;
    }

    @Override
    public void periodic() {
        intakeVelocityPub.set(m_motorIntake.getVelocity().getValueAsDouble());
        intakingPub.set(intaking);
    }
}
