// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class ControllerConstants {
                public static final int kDriverPort = 0;
                public static final int kOperatorPort = 1;
                public static final double kDeadband = 0.1;
        }

        public static final class DrivetrainConstants {
                // The desired top speed of the robot.
                public final static LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts.div(8);
                // The maximum turning rate (in radians per second).
                public final static AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.5);
        }

        public static final class ShooterConstants {
                // The motor ID of the left motor.
                public final static int LeftMotorId = 35;
                // The motor ID of the right motor.
                public final static int RightMotorId = 36;

                // max rps for passing
                public final static int maxRPS = 100;

                // rps at far distance is 50rps
                // rps at climber center is 47 rps
                public final static AngularVelocity ShootRPS = RotationsPerSecond.of(47.0);

                /// The PID settings for the shooter motors.
                public final static Slot0Configs ShooterGains = new Slot0Configs()
                                .withKP(0.41).withKI(0).withKD(0.00165)
                                .withKS(0).withKV(0.123);

                // The strength of gravity
                public final static LinearAcceleration G = MetersPerSecondPerSecond.of(9.81);

                // The distance from floor to hub target point
                public final static Distance HubTargetHeight = Meters.of(1.524);
                /// The vertical position of ball extake
                public final static Distance BallExtakeHeight = Meters.of(0.432);
                /// The angle at which the shooter is mounted above the horizontal.
                public final static Angle Pitch = Degrees.of(73);

                /// The scaling constant to correct for damping
                public final static double DampingCoefficient = 1.82;
        }

        public static final class IntakeConstants {
                // The motor ID of the intake motor.
                public final static int IntakeMotorId = 37;

                public final static AngularVelocity IndexRPS = RotationsPerSecond.of(20);

                public final static double GearRatio = 5.0;

                // UNTUNED
                // The PID settings for the intake motor.
                public final static Slot0Configs IntakeGains = new Slot0Configs()
                                .withKP(0.5).withKI(0).withKD(0)
                                .withKS(0).withKV(0.15);

                public final static AngularVelocity IntakeRPS = RotationsPerSecond.of(2);
        }

        public static final class SlapdownConstants {
                // The motor ID of the slapdown motor.
                public final static int SlapdownMotorId = 104;

                /// The position to lower the slapdown to when intaking a ball.
                public final static Angle IntakePosition = Rotations.of(0.25);
                /// The default position of slapdown system.
                public final static Angle HomePosition = Rotations.of(0);

                /// The PID settings for the slapdown motor.
                public final static Slot0Configs SlapdownGains = new Slot0Configs()
                                .withKP(0.1).withKI(0).withKD(0)
                                .withKS(0).withKV(0.1);
        }

        public static final class KickerConstants {
                // The motor ID of the kicker motor.
                public final static int KickerMotorId = 105;

                /// The PID settings for the kicker motor.
                ///
                /// UNTUNED
                public final static Slot0Configs KickerGains = new Slot0Configs()
                                .withKP(0.1).withKI(0).withKD(0)
                                .withKS(0).withKV(0.1);
        }

        public class TunerConstants {
                // Both sets of gains need to be tuned to your individual robot.

                // The steer motor uses any SwerveModule.SteerRequestType control request with
                // the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                                .withKP(50).withKI(0).withKD(0.5)
                                .withKS(0.1).withKV(1.00).withKA(0)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                                .withKP(0.15).withKI(0).withKD(0)
                                .withKS(0).withKV(0.124);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The type of motor used for the drive motor
                private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
                // The type of motor used for the drive motor
                private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

                // The remote sensor feedback type to use for the steer motors;
                // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
                private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final Current kSlipCurrent = Amps.of(120);

                // Initial configs for the drive and steer motors and the azimuth encoder; these
                // cannot be null.
                // Some configs will be overwritten; check the `with*InitialConfigs()` API
                // documentation.
                private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
                private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                                .withCurrentLimits(
                                                new CurrentLimitsConfigs()
                                                                // Swerve azimuth does not require much torque output,
                                                                // so we can set a relatively low
                                                                // stator current limit to help avoid brownouts without
                                                                // impacting performance.
                                                                .withStatorCurrentLimit(Amps.of(60))
                                                                .withStatorCurrentLimitEnable(true)
                                                                .withSupplyCurrentLimit(Amps.of(20))
                                                                .withSupplyCurrentLimitEnable(true));

                private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
                // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
                private static final Pigeon2Configuration pigeonConfigs = null;

                // CAN bus that the devices are located on;
                // All swerve devices must share the same CAN bus
                public static final CANBus kCANBus = new CANBus("Drive SubSystem CANivore", "./logs/example.hoot");

                // Theoretical free speed (m/s) at 12 V applied output;
                // This needs to be tuned to your individual robot
                public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(9.23);

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 0;

                private static final double kDriveGearRatio = 6.23;
                private static final double kSteerGearRatio = 25;
                private static final Distance kWheelRadius = Inches.of(3.725);

                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                private static final int kPigeonId = 15;

                // These are only used for simulation
                private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
                private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
                // Simulated voltage necessary to overcome friction
                private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
                private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

                public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withCANBusName(kCANBus.getName())
                                .withPigeon2Id(kPigeonId)
                                .withPigeon2Configs(pigeonConfigs);

                private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withWheelRadius(kWheelRadius)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                                .withSlipCurrent(kSlipCurrent)
                                .withSpeedAt12Volts(kSpeedAt12Volts)
                                .withDriveMotorType(kDriveMotorType)
                                .withSteerMotorType(kSteerMotorType)
                                .withFeedbackSource(kSteerFeedbackType)
                                .withDriveMotorInitialConfigs(driveInitialConfigs)
                                .withSteerMotorInitialConfigs(steerInitialConfigs)
                                .withEncoderInitialConfigs(encoderInitialConfigs)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage);

                // Front Left --> Back Left
                private static final int kBackLeftDriveMotorId = 2;
                private static final int kBackLeftSteerMotorId = 1;
                private static final int kBackLeftEncoderId = 10;
                private static final Angle kBackLeftEncoderOffset = Rotations.of(0.132080078125);
                private static final boolean kBackLeftSteerMotorInverted = false;
                private static final boolean kBackLeftEncoderInverted = false;

                private static final Distance kBackLeftXPos = Inches.of(11.25);
                private static final Distance kBackLeftYPos = Inches.of(12);

                // Front Right --> Front Left
                private static final int kFrontLeftDriveMotorId = 4;
                private static final int kFrontLeftSteerMotorId = 3;
                private static final int kFrontLeftEncoderId = 11;
                private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.40234375);
                private static final boolean kFrontLeftSteerMotorInverted = false;
                private static final boolean kFrontLeftEncoderInverted = false;

                private static final Distance kFrontLeftXPos = Inches.of(11.25);
                private static final Distance kFrontLeftYPos = Inches.of(-12);

                // Back Left --> Back Right
                private static final int kBackRightDriveMotorId = 8;
                private static final int kBackRightSteerMotorId = 7;
                private static final int kBackRightEncoderId = 13;
                private static final Angle kBackRightEncoderOffset = Rotations.of(0.331298828125);
                private static final boolean kBackRightSteerMotorInverted = false;
                private static final boolean kBackRightEncoderInverted = false;

                private static final Distance kBackRightXPos = Inches.of(-11.25);
                private static final Distance kBackRightYPos = Inches.of(12);

                // Back Right --> Front Right
                private static final int kFrontRightDriveMotorId = 6;
                private static final int kFrontRightSteerMotorId = 5;
                private static final int kFrontRightEncoderId = 12;
                private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.31396484375);
                private static final boolean kFrontRightSteerMotorInverted = false;
                private static final boolean kFrontRightEncoderInverted = false;

                private static final Distance kFrontRightXPos = Inches.of(-11.25);
                private static final Distance kFrontRightYPos = Inches.of(-12);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                                .createModuleConstants(
                                                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                                                kFrontLeftEncoderOffset,
                                                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide,
                                                kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                                .createModuleConstants(
                                                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                                                kFrontRightEncoderOffset,
                                                kFrontRightXPos, kFrontRightYPos, kInvertRightSide,
                                                kFrontRightSteerMotorInverted, kFrontRightEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                                .createModuleConstants(
                                                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                                                kBackLeftEncoderOffset,
                                                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide,
                                                kBackLeftSteerMotorInverted, kBackLeftEncoderInverted);
                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                                .createModuleConstants(
                                                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                                                kBackRightEncoderOffset,
                                                kBackRightXPos, kBackRightYPos, kInvertRightSide,
                                                kBackRightSteerMotorInverted, kBackRightEncoderInverted);

                /**
                 * Creates a CommandSwerveDrivetrain instance.
                 * This should only be called once in your robot program,.
                 */
                public static CommandSwerveDrivetrain createDrivetrain() {
                        return new CommandSwerveDrivetrain(
                                        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
                }

                /**
                 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected
                 * device types.
                 */
                public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
                         * @param modules             Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, modules);
                        }

                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
                         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
                         *                                unspecified or set to 0 Hz, this is 250 Hz on
                         *                                CAN FD, and 100 Hz on CAN 2.0.
                         * @param modules                 Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        double odometryUpdateFrequency,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, odometryUpdateFrequency, modules);
                        }

                        /**
                         * Constructs a CTRE SwerveDrivetrain using the specified constants.
                         * <p>
                         * This constructs the underlying hardware devices, so users should not
                         * construct
                         * the devices themselves. If they need the devices, they can access them
                         * through
                         * getters in the classes.
                         *
                         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
                         *                                  drive
                         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
                         *                                  unspecified or set to 0 Hz, this is 250 Hz
                         *                                  on
                         *                                  CAN FD, and 100 Hz on CAN 2.0.
                         * @param odometryStandardDeviation The standard deviation for odometry
                         *                                  calculation
                         *                                  in the form [x, y, theta]áµ€, with units in
                         *                                  meters
                         *                                  and radians
                         * @param visionStandardDeviation   The standard deviation for vision
                         *                                  calculation
                         *                                  in the form [x, y, theta]áµ€, with units in
                         *                                  meters
                         *                                  and radians
                         * @param modules                   Constants for each specific module
                         */
                        public TunerSwerveDrivetrain(
                                        SwerveDrivetrainConstants drivetrainConstants,
                                        double odometryUpdateFrequency,
                                        Matrix<N3, N1> odometryStandardDeviation,
                                        Matrix<N3, N1> visionStandardDeviation,
                                        SwerveModuleConstants<?, ?, ?>... modules) {
                                super(
                                                TalonFX::new, TalonFX::new, CANcoder::new,
                                                drivetrainConstants, odometryUpdateFrequency,
                                                odometryStandardDeviation, visionStandardDeviation, modules);
                        }
                }
        }

        public static final class Vision {
                public static final String kCameraName = "YOUR CAMERA NAME";
                // Cam mounted facing forward, half a meter forward of center, half a meter up
                // from center,
                // pitched upward.
                private static final double camPitch = Units.degreesToRadians(30.0);
                public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                new Rotation3d(0, -camPitch, 0));

                // The layout of the AprilTags on the field
                public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                .loadField(AprilTagFields.kDefaultField);

                // The standard deviations of our vision estimated poses, which affect
                // correction rate
                // (Fake values. Experiment and determine estimation noise on an actual robot.)
                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }

        public static final class GameConstants {
                public static final Translation2d blueHubLocation = new Translation2d(4.03, 4.035);
                public static final Translation2d redHubLocation = new Translation2d(12.51, 4.035);
        }
}