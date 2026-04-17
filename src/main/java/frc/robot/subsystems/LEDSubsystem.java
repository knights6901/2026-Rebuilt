package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.LEDConstants.Length;
import static frc.robot.Constants.LEDConstants.Port;
import static frc.robot.Constants.LEDConstants.RainbowPattern;
import static frc.robot.Constants.LEDConstants.ScrollRaindbowPattern;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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

        // led.start();

        // setDefaultCommand(new RunCommand(() -> {
        // double vx = drivetrain.getState().Speeds.vxMetersPerSecond;
        // double vy = drivetrain.getState().Speeds.vyMetersPerSecond;

        // double speed = Math.sqrt(vx * vx + vy * vy);
        // double maxSpeed = DrivetrainConstants.MaxSpeed.in(MetersPerSecond)
        // * DrivetrainConstants.TeleopMovementSensitivity;

        // double percent = speed / maxSpeed;

        // LEDPattern rainbow = LEDConstants.RainbowPattern
        // .scrollAtRelativeSpeed(Percent.per(Second).of(20));

        // LEDPattern purple = LEDPattern.solid(Color.kPurple)
        // .scrollAtRelativeSpeed(Percent.per(Second).of(20));

        // // Mask that alternates every other LED (0=show base, 1=show overlay)
        // LEDPattern mask = LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.5,
        // Color.kBlack))
        // .scrollAtRelativeSpeed(Percent.per(Second).of(20));

        // purple.mask(mask).overlayOn(rainbow).applyTo(buffer);
        // }, this));

        // setDefaultCommand(new RunCommand(() -> {
        //     long t = System.currentTimeMillis();
        //     double scroll = (t % 5000) / 5000.0; // full cycle every 5 seconds

        //     for (int i = 0; i < Length; i++) {
        //         // offset by scroll amount
        //         double pos = ((i / (double) Length) + scroll) % 1.0;
        //         int segment = (int)(pos * 4) % 4;

        //         if (segment % 2 == 0) {
        //             buffer.setLED(i, Color.kPurple);
        //         } else {
        //             // rainbow hue based on position within segment
        //             double hue = (pos * 2 % 1.0) * 180.0;
        //             buffer.setHSV(i, (int) hue, 255, 255);
        //         }
        //     }
        // }, this));

        // setDefaultCommand(runPattern(ScrollRaindbowPattern));
    }

    /* sketchy claude code */
    /* update, it works */
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
        // return run(() -> pattern.applyTo(buffer));
        return run(() -> {
        });
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

    public Command runAllPatterns(LEDPattern patternLeft, LEDPattern patternMiddle, LEDPattern patternRight) {
        return new ParallelCommandGroup(
            runPatternLeft(patternLeft),
            runPatternMiddle(patternMiddle),
            runPatternRight(patternRight),
            new RunCommand(() -> {}, this)
        );
    }

    public LEDPattern shooterPattern(Supplier<AngularVelocity> current, AngularVelocity target) {
        return fireUpPattern(current.get().in(RotationsPerSecond) / target.in(RotationsPerSecond));
    }

    // @Override
    // public void periodic() {
    // led.setData(buffer);
    // }
}