// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    public static final class LoggingConstants {
        public static final boolean kLogging = true;
    }

    public static final class TuningModeConstants {
        public static final boolean kTuning = true;
    }

    public static final class IntakeConstants {
        public static final int kIRPort = 1;
        public static final int kIntakeCanId = 21;
        public static final double speed = .5;
    }

    public static final class ShooterConstants {
        public static final int kShooter1CanId = 26;
        public static final int kShooter2CanId = 27;
        public static final int kShooterMotor2CurrentLimit = 0;
        public static final IdleMode kShooterMotor2IdleMode = null;
        public static final int kShooterMotor1CurrentLimit = 0;
        public static IdleMode kShooterMotor1IdleMode = IdleMode.kBrake;
        public static double kShooter2MaxOutput = 0.8;
        public static double kShooter2MinOutput = -0.8;
        public static double kShooter2FF = 0;
        public static double kShooter2P = 1;
        public static double kShooter2I = 0.0001;
        public static double kShooter2D = 0;
        public static double kShooter1MaxOutput = 0.8;
        public static double kShooter1MinOutput = -0.8;
        public static double kShooter1FF = 0;
        public static double kShooter1P = 1;
        public static double kShooter1I = 0.0001;
        public static double kShooter1D = 0;
        public static double kShooterEncoder2VelocityFactor = (2 * Math.PI) / 60.0;
        public static double kShooterEncoder2PositionFactor = (2 * Math.PI);
        public static double kshooterEncoder1VelocityFactor = (2 * Math.PI) / 60.0;
        public static double kshooterEncoder1PositionFactor = (2 * Math.PI);
    }

    public static final class ArmConstants {
        public static final int kArmCanId = 23;
        public static final int kShooterArmCanId = 24;

        public static final int kArmMotorCurrentLimit = 20; // amps
        public static final int kShooterArmMotorCurrentLimit = 20; // amps

        // Invert the encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kArmEncoderInverted = true;
        public static final boolean kShooterArmEncoderInverted = true;

        public static final double kArmEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kArmEncoderPositionPIDMinInput = 0; // radians
        public static final double kArmEncoderPositionPIDMaxInput = kArmEncoderPositionFactor; // radians

        public static final double kShooterArmEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kShooterArmEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kShooterArmEncoderPositionPIDMinInput = 0; // radians
        public static final double kShooterArmEncoderPositionPIDMaxInput = kShooterArmEncoderPositionFactor; // radians

        public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kShooterArmMotorIdleMode = IdleMode.kBrake;

        public static final double kArmMaxSpeed = 0.3;
        public static final double kShooterArmMaxSpeed = 0.3;

        // PIDS
        // Arm PID coefficients
        public static final int kArmPIDSlot = 0;
        public static final double kArmP = 1;
        public static final double kArmI = 0.0001;
        public static final double kArmD = 0.1;
        public static final double kArmFF = 0;
        public static final double kArmMaxOutput = kArmMaxSpeed;
        public static final double kArmMinOutput = -kArmMaxSpeed;

        // Shooter PID coefficients
        public static final int kShooterArmPIDSlot = 0;
        public static final double kShooterArmP = .5;
        public static final double kShooterArmI = 0.0001;
        public static final double kShooterArmD = 0.1;
        public static final double kShooterArmFF = 0;
        public static final double kShooterArmMaxOutput = kShooterArmMaxSpeed;
        public static final double kShooterArmMinOutput = -kShooterArmMaxSpeed;

        // SETPOINTS

        public static final double kAdjustmentFactor = 90;

        public enum Position {
            TRAVEL("Travel", 35 - kAdjustmentFactor, 83),
            PODIUM("Podium", 46 - kAdjustmentFactor, 79),
            LOW_SUBWOOFER("LowSubwoofer", 75 - kAdjustmentFactor, 75);

            private static final Map<String, Position> BY_LABEL = new HashMap<>();
            private static final Map<Double, Position> BY_ARM_POSITION = new HashMap<>();
            private static final Map<Double, Position> BY_SHOOTER_ARM_POSITION = new HashMap<>();

            static {
                for (Position e : values()) {
                    BY_LABEL.put(e.label, e);
                    BY_ARM_POSITION.put(e.armPosition, e);
                    BY_SHOOTER_ARM_POSITION.put(e.shooterArmPosition, e);
                }
            }

            public final String label;
            public final double armPosition;
            public final double shooterArmPosition;

            private Position(String label, double armPosition, double shooterArmPosition) {
                this.label = label;
                this.armPosition = armPosition;
                this.shooterArmPosition = shooterArmPosition;
            }

            public static Position valueOfLabel(String label) {
                return BY_LABEL.get(label);
            }

            public static Position valueOfArmPosition(double armPosition) {
                return BY_ARM_POSITION.get(armPosition);
            }

            public static Position valueOfShooterArmPosition(double shooterArmPosition) {
                return BY_SHOOTER_ARM_POSITION.get(shooterArmPosition);
            }

        }

        public static final double kTravelArm = 35 - kAdjustmentFactor;
        public static final double kTravelShooter = 83;

        // podium
        public static final double kPodiumArm = 46 - kAdjustmentFactor;
        public static final double kPodiumShooter = 79;

        // low subwoofer
        public static final double kLowSubwooferArm = 75 - kAdjustmentFactor;
        public static final double kLowSubwooferShooter = 75;

        // amp
        public static final double kAmpArm = 114 - kAdjustmentFactor;
        public static final double kAmpShooter = -39;

        // trap approach
        public static final double kTrapApproachArm = 132 - kAdjustmentFactor;
        public static final double kTrapApproachShooter = -42;

        // trap climb
        public static final double kTrapClimbArm = 130 - kAdjustmentFactor;
        public static final double kTrapClimbShooter = -40;

        // trap score
        public static final double kTrapScoreArm = 130 - kAdjustmentFactor;
        public static final double kTrapScoreShooter = -55;

        // high podium
        public static final double kHighPodiumArm = 127 - kAdjustmentFactor;
        public static final double kHighPodiumShooter = -40;

        // back podium
        public static final double kBackPodiumArm = 127 - kAdjustmentFactor;
        public static final double kBackPodiumShooter = 166;

        // high subwoofer
        public static final double kHighSubwooferArm = 127 - kAdjustmentFactor;
        public static final double kHighSubwooferShooter = 23;

        // Estimates, fix this once we get exact measurements
        public static final double kArmTotalDegrees = 72.4; // TODO
        public static final double kArmTotalRevolutions = 5.488; // TODO

        // Convert angle of travel to encoder rotations, where encoder reading of .1 is
        // 0 degrees and reading of 5.5 is 90 degrees
        public static final double kArmRevolutionsPerDegree = -(kArmTotalRevolutions)
                / kArmTotalDegrees;

        // Estimates, fix this once we get exact measurements
        public static final double kShooterArmTotalDegrees = 72.4; // TODO
        public static final double kShooterArmTotalRevolutions = 5.488; // TODO

        // Convert angle of travel to encoder rotations, where encoder reading of .1 is
        // 0 degrees and reading of 5.5 is 90 degrees
        public static final double kShooterArmRevolutionsPerDegree = -(kShooterArmTotalRevolutions)
                / kShooterArmTotalDegrees;
    }

    public static final class ClimberConstants {
        public static final int kWinchCanId = 22;
    }

    public static final class FeederConstants {
        public static final int kFeederCanId = 25;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(24.0);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.0);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 3;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 13;
        public static final int kRearRightTurningCanId = 14;

        // Gyro Constants
        public static final int kGyroDeviceNumber = 15;
        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
        public static int kOperatorControllerPort = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
