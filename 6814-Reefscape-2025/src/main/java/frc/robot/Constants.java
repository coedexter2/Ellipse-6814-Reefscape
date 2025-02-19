package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class OuttakeConstants {

        public static final int kOuttakeMotorPort = 0;
        public static final int kBeamBreakPort = 0;
        public static final double kOuttakeSpeed = .1;
        public static final double kIntakeSpeed = .1;
        public static final double kOuttakeTimeout = 1;

        public static final SparkMaxConfig kOuttakeMotorConfig = new SparkMaxConfig();
    
    }
    public static final class ClimbConstants {

        public static final int kClimbMotorID = 0;
        public static final int kLimitSwitchPort = 0;
        public static final SparkMaxConfig kClimbMotorConfig = new SparkMaxConfig();
        public static final double kClimbEncoderRotToMeters = 0.0; // fix this
    
        public static final double kClimbSpeed = 1;
        public static final double kClimbSetpoint = 0;
    }

    public static final class ElevatorConstants {

        public static final int kElevatorMotorPort = 0;

        public static final double kPulleyDiameterMeter = Units.inchesToMeters(1.504);
        public static final double kElevatorMotorGearRatio = 1 / 15;
        public static final double kBeltPullMeters = kElevatorMotorGearRatio * Math.PI * kPulleyDiameterMeter;
        public static final double kElevatorEncoderRot2Meters = 2 * kBeltPullMeters;

        public static final SparkMaxConfig kElevatorMotorConfig = new SparkMaxConfig();

        // FEEDFORWARD CONSTANTS
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kMaxVelocity = 0.0;
        public static final double kMaxAcceleration = 0.0;

        // PID CONSTANTS
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);


        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(25.125);
        // Distance between right and left wheels
        public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(29);
        // Distance between front and back wheels
        public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kDriveKinematics = new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
            new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        /*
         * === Welcome to the "Which module is front left?" Center!
         * 
         *   FRONT LEFT:  Module 0
        *   BACK LEFT:   Module 1
        *   FRONT RIGHT: Module 2
         *   BACK RIGHT:  Module 3
        * 
        *   Swerve drive debugging stats:
        *   2025: 2 hours
        */

        public static final int kFrontLeftDriveMotorPort = 7; //mod0
        public static final int kBackLeftDriveMotorPort = 1;//mod1
        public static final int kFrontRightDriveMotorPort = 5; //mod2
        public static final int kBackRightDriveMotorPort = 3;//mod3

        public static final int kFrontLeftTurningMotorPort = 8;//mod0
        public static final int kBackLeftTurningMotorPort = 2;//mod1
        public static final int kFrontRightTurningMotorPort = 6; //mod2
        public static final int kBackRightTurningMotorPort = 4;//mod3

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 0;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.182; //module 2
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.356; //module 3
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.618; //module 1
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.362; //module 0

        public static final double kPhysicalMaxSpeedMetersPerSecond = 8;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2 ;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 0;
        public static final int kDriverXAxis = 1;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
}

public static final class FeildConstants
{
    public static final int kBlueMidpointCords[][] = {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}};
    public static final int kRedMidpointCords[][] = {{1,1},{1,1},{1,1},{1,1},{1,1},{1,1}};
}

}