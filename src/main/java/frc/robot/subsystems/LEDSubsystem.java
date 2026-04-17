package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.LEDConstants.Length;
import static frc.robot.Constants.LEDConstants.Port;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView left;
    private final AddressableLEDBufferView middle;
    private final AddressableLEDBufferView right;

    public LEDSubsystem(CommandSwerveDrivetrain drivetrain) {
        led = new AddressableLED(Port);
        buffer = new AddressableLEDBuffer(Length);
        led.setLength(Length);

        left = buffer.createView(0, Length / 3);
        middle = buffer.createView(Length / 3, 2 * Length / 3);
        right = buffer.createView(2 * Length / 3 + 1, Length - 1);

        led.start();
    }

    public LEDPattern fireUpPattern(double t) {
        t = Math.max(0.0, Math.min(1.0, t)); // clamp

        if (t < 0.33) {
            // red → orange
            double local = t / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 * local, 0.0));
        } else if (t < 0.66) {
            // orange → yellow
            double local = (t - 0.33) / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 + 0.73 * local, 0.0));
        } else {
            // yellow → green
            double local = (t - 0.66) / 0.34;
            return LEDPattern.solid(new Color(1.0 - local, 1.0, 0.0));
        }
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    public Command runPatternLeft(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(left));
    }

    public Command runPatternMiddle(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(middle));
    }

    public Command runPatternRight(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(right));
    }

    public Command runAllPatterns(
        Supplier<LEDPattern> patternLeft, 
        Supplier<LEDPattern> patternMiddle, 
        Supplier<LEDPattern> patternRight) {
        return run(() -> {
            patternLeft.get().applyTo(left);
            patternMiddle.get().applyTo(middle);
            patternRight.get().applyTo(right);
            // pope naveen sai joseph dwane the rock johnsohn kizhcackel the viola the third of the holy roman empire has blessed this code
        });
    }

    public LEDPattern shooterPattern(Supplier<AngularVelocity> current, AngularVelocity target) {
        return fireUpPattern(current.get().in(RotationsPerSecond) / target.in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }
}