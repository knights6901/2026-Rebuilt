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
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        /**
         * Controller-related constants for gamepad input.
         */
        public static final class ControllerConstants {
                /** The USB port number for the driver controller. */
                public static final int DriverPort = 0;
                /** The USB port number for the operator controller. */
                public static final int OperatorPort = 1;
                /** The deadband threshold for controller joysticks (0-1 scale). */
                public static final double Deadband = 0.1;
        }

        public static final class DrivetrainConstants {
                /** The desired top speed of the robot in meters per second. */
                public final static LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
                /** The movement sensitivity multiplier for teleop control (between 0 and 1). */
                public final static double TeleopMovementSensitivity = .76901;
                /** The maximum angular turning rate in rotations per second. */
                public final static AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.5);
        }

        public static final class ShooterConstants {
                /** The CAN ID of the left shooter motor. */
                public final static int LeftMotorId = 35;
                /** The CAN ID of the right shooter motor. */
                public final static int RightMotorId = 36;

                /** The distance from the center of the robot to the shooter (horizontally). */
                public final static Distance CenterToShooter = Inches.of(8);

                /** The maximum rotations per second that the shooter can achieve. */
                public final static AngularVelocity MaxRPS = RotationsPerSecond.of(80);
                /**
                 * The default rotations per second of the shooter to shoot a ball (tested
                 * experimentally).
                 */
                public final static AngularVelocity DefaultRPS = RotationsPerSecond.of(30.6901);

                /**
                 * The tolerance for determining whether the shooter is "primed" and ready to
                 * shoot.
                 */
                public final static AngularVelocity PrimingTolerance = RotationsPerSecond.of(3);

                /** The default prime RPS for the shooter. */
                public final static AngularVelocity DefaultPrimeRPS = RotationsPerSecond.of(40);

                /** The PID and feedforward settings for the shooter motors. */
                public final static Slot0Configs Gains = new Slot0Configs()
                                .withKP(0.36901).withKI(0).withKD(0.0085)
                                .withKS(0).withKV(0.119);

                /** The strength of gravity (9.81 m/s²). */
                public final static LinearAcceleration G = MetersPerSecondPerSecond.of(9.81);

                /** The vertical position of the ball exit point from the shooter. */
                public final static Distance BallExtakeHeight = Meters.of(0.432);
                /** The angle at which the shooter is mounted above the horizontal plane. */
                public final static Angle Pitch = Degrees.of(76);

                /**
                 * The scaling constant to correct for damping in the shooter mechanism when the
                 * robot is "far" from the hub.
                 */
                public final static double DampingFarCoefficient = 1.6901 + 0.02;
                /**
                 * The scaling constant to correct for damping in the shooter mechanism when the
                 * robot is "near" from the hub.
                 */
                public final static double DampingNearCoefficient = 1.6901 + 0.1;

                /** The maximum distance to be considered "near" to the hub. */
                public final static Distance NearHubDistance = Meters.of(2);

                /** The complete motor configuration for the shooter system. */
                public static final TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                                .withSlot0(ShooterConstants.Gains)
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.CounterClockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(Amps.of(40))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(60))
                                                .withSupplyCurrentLimitEnable(true));
        }

        public static final class IndexerConstants {
                /** The CAN ID of the indexer motor. */
                public final static int MotorId = 40;

                /** The target rotations per second for the indexer motor during operation. */
                public final static AngularVelocity Power = RotationsPerSecond.of(85);

                /** The complete motor configuration for the indexer system. */
                public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.CounterClockwise_Positive));
                // .withCurrentLimits(new CurrentLimitsConfigs()
                // .withStatorCurrentLimit(Amps.of(40))
                // .withStatorCurrentLimitEnable(true)
                // .withSupplyCurrentLimit(Amps.of(60))
                // .withSupplyCurrentLimitEnable(true));
        }

        public static final class IntakeConstants {
                /** The CAN ID of the intake motor. */
                public final static int MotorId = 32;

                /** The rotations per second for actively intaking balls. */
                public final static AngularVelocity IntakeRPS = RotationsPerSecond.of(80);

                /** The gear ratio of the intake system. */
                public final static double GearRatio = 9.0;

                /** The PID and feedforward settings for the intake motor. */
                public final static Slot0Configs Gains = new Slot0Configs()
                                .withKP(0.5).withKI(0).withKD(0)
                                .withKS(0).withKV(0.15);

                /** The complete motor configuration for the intake system. */
                public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                                .withSlot0(IntakeConstants.Gains)
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(Amps.of(40))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(60))
                                                .withSupplyCurrentLimitEnable(true));
        }

        public static final class SlapdownConstants {
                /** The CAN ID of the slapdown motor. */
                public final static int MotorId = 31;

                /** The position to lower the slapdown to when intaking a ball. */
                public final static Angle IntakePosition = Rotations.of(56);
                /** The default home position of slapdown system. */
                public final static Angle HomePosition = Rotations.of(0);
                /**
                 * The tolerance for determining whether the slapdown is in the deployed
                 * position.
                 */
                public final static Angle PositionTolerance = Rotations.of(2.0);

                /** The gear ratio of the slapdown system. */
                public final static double GearRatio = 81.0;

                /** The PID and feedforward settings for the slapdown motor. */
                public final static Slot0Configs Gains = new Slot0Configs()
                                .withKP(0.4).withKI(0).withKD(0.1)
                                .withKS(0).withKV(1.3);

                /** The complete motor configuration for the slapdown system. */
                public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                                .withSlot0(SlapdownConstants.Gains)
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.CounterClockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(Amps.of(40))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(60))
                                                .withSupplyCurrentLimitEnable(true));
        }

        public static final class KickerConstants {
                /** The CAN ID of the kicker motor. */
                public final static int MotorId = 37;

                public final static double KickerPower = 0.8506901;

                /** The complete motor configuration for the kicker system. */
                public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(Amps.of(40))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(60))
                                                .withSupplyCurrentLimitEnable(true));
        }

        // Generated by the 2026 Tuner X Swerve Project Generator
        // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
        public class TunerConstants {
                // Both sets of gains need to be tuned to your individual robot.

                // The steer motor uses any SwerveModule.SteerRequestType control request with
                // the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                                .withKP(100).withKI(0).withKD(0.5)
                                .withKS(0.1).withKV(2).withKA(0)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                                .withKP(0.1).withKI(0).withKD(0)
                                .withKS(0).withKV(0.1);

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
                private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(Amps.of(80))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(40))
                                                .withSupplyCurrentLimitEnable(true));

                private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                // Swerve azimuth does not require much torque output,
                                                // so we can set a relatively low
                                                // stator current limit to help avoid brownouts without
                                                // impacting performance.
                                                .withStatorCurrentLimit(Amps.of(40))
                                                .withStatorCurrentLimitEnable(true)
                                                .withSupplyCurrentLimit(Amps.of(20))
                                                .withSupplyCurrentLimitEnable(true));

                private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
                // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
                private static final Pigeon2Configuration pigeonConfigs = null;

                // CAN bus that the devices are located on;
                // All swerve devices must share the same CAN bus
                public static final CANBus kCANBus = new CANBus("Swerve CANivore", "./logs/example.hoot");

                // Theoretical free speed (m/s) at 12 V applied output;
                // This needs to be tuned to your individual robot
                public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.83);

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 0;

                private static final double kDriveGearRatio = 6.23;
                private static final double kSteerGearRatio = 25;
                private static final Distance kWheelRadius = Inches.of(1.95);

                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;

                public static final int kPigeonId = 15;

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

                // Front Left
                private static final int kFrontLeftDriveMotorId = 2;
                private static final int kFrontLeftSteerMotorId = 1;
                private static final int kFrontLeftEncoderId = 10;
                private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.33837890625);
                private static final boolean kFrontLeftSteerMotorInverted = false;
                private static final boolean kFrontLeftEncoderInverted = false;

                private static final Distance kFrontLeftXPos = Inches.of(11.25);
                private static final Distance kFrontLeftYPos = Inches.of(12);

                // Front Right
                private static final int kFrontRightDriveMotorId = 4;
                private static final int kFrontRightSteerMotorId = 3;
                private static final int kFrontRightEncoderId = 11;
                private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.06396484375);
                private static final boolean kFrontRightSteerMotorInverted = false;
                private static final boolean kFrontRightEncoderInverted = false;

                private static final Distance kFrontRightXPos = Inches.of(11.25);
                private static final Distance kFrontRightYPos = Inches.of(-12);

                // Back Left
                private static final int kBackLeftDriveMotorId = 8;
                private static final int kBackLeftSteerMotorId = 7;
                private static final int kBackLeftEncoderId = 13;
                private static final Angle kBackLeftEncoderOffset = Rotations.of(0.39208984375);
                private static final boolean kBackLeftSteerMotorInverted = false;
                private static final boolean kBackLeftEncoderInverted = false;

                private static final Distance kBackLeftXPos = Inches.of(-11.25);
                private static final Distance kBackLeftYPos = Inches.of(12);

                // Back Right
                private static final int kBackRightDriveMotorId = 6;
                private static final int kBackRightSteerMotorId = 5;
                private static final int kBackRightEncoderId = 12;
                private static final Angle kBackRightEncoderOffset = Rotations.of(0.072265625);
                private static final boolean kBackRightSteerMotorInverted = false;
                private static final boolean kBackRightEncoderInverted = false;

                private static final Distance kBackRightXPos = Inches.of(-11.25);
                private static final Distance kBackRightYPos = Inches.of(-12);

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

        /**
         * Vision processing constants for AprilTag-based localization.
         */
        public static final class Vision {
                /** The name/identifier of the camera used for vision processing. */
                public static final String kCameraName = "YOUR CAMERA NAME";

                /** The pitch angle of the camera relative to the horizontal plane. */
                private static final double camPitch = Units.degreesToRadians(30.0);

                /**
                 * The 3D transformation from the robot center to the camera position and
                 * orientation.
                 */
                public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                                new Rotation3d(0, -camPitch, 0));

                /** The layout of AprilTags on the field for localization. */
                public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                .loadField(AprilTagFields.kDefaultField);

                /** Standard deviations for single AprilTag pose estimation (x, y, theta). */
                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                /** Standard deviations for multi-AprilTag pose estimation (x, y, theta). */
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }

        /**
         * Game field-related constants.
         */
        public static final class GameConstants {
                /** The position of the hub/target on the blue alliance side of the field. */
                public static final Translation2d BlueHubLocation = new Translation2d(4.612, 4.021);
                /** The position of the hub/target on the red alliance side of the field. */
                public static final Translation2d RedHubLocation = new Translation2d(11.901, 4.021);

                /** The y-position of the left trench on the blue alliance side of the field. */
                public static final Distance BlueLeftTrenchY = Meters.of(7.435);
                /**
                 * The y-position of the right trench on the blue alliance side of the field.
                 */
                public static final Distance BlueRightTrenchY = Meters.of(0.634);

                /** The x-position of the pass-line on the blue alliance side of the field. */
                public static final Distance BluePassLineX = Meters.of(2.306);
                /** The x-position of the pass-line on the red alliance side of the field. */
                public static final Distance RedPassLineX = Meters.of(14.207);

                /** The height of the target hub from the ground. */
                public final static Distance HubTargetHeight = Meters.of(1.524);

                /** Returns the position of the hub based on the current alliance. */
                public static Translation2d getHubLocation() {
                        return (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) ? BlueHubLocation
                                        : RedHubLocation;
                }

                /**
                 * Returns the position of the pass line based on the current alliance and
                 * drivetrain position.
                 */
                public static Translation2d getPassLocation(Pose2d drivetrainPose) {
                        Distance y = drivetrainPose.getMeasureY();

                        return (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
                                        ? new Translation2d(GameConstants.BluePassLineX, y)
                                        : new Translation2d(GameConstants.RedPassLineX, y);
                }
        }

        public static final class LEDConstants {
                /** The PWM port that the led bus is connected to the RIO on. */
                public static final int Port = 0;
                /** The length (in number of LED connections on the strip). */
                public static final int Length = 20;
        }
}