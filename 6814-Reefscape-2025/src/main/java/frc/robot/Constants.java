package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class OuttakeConstants {

        public static final int kOuttakeMotorPort = 31;
        public static final int kBeamBreakPort = 0;
        public static final double kOuttakeSpeed = -1.0;
        public static final double kIntakeSpeed = -0.3;
        public static final double kOuttakeTimeout = 1;

        public static SparkMaxConfig kOuttakeMotorConfig = new SparkMaxConfig();
    
    }
    public static final class ClimbConstants {
        public static final int kClimbMotorID = 20;
        public static final int kLimitSwitchPort = 1;
        public static final SparkMaxConfig kClimbMotorConfig = new SparkMaxConfig();

        public static final double kClimbMotorGearRatio = (1.0 / 500.0);
        public static final double kClimbEncoderRotToRadians = kClimbMotorGearRatio * 2 * Math.PI; // fix this
    
        public static final double kClimbHomeSpeed = 0.2;
        public static final double kClimbSpeed = 0.5;
        public static final double kClimbSetpoint = Math.toRadians(125);
    }

    public static final class ElevatorConstants {

        public static final int kElevatorMotorPort = 30;
        public static final int kElevatorLimitSwitchPort = 9;

        public static final double kPulleyDiameterMeter = Units.inchesToMeters(1.504);
        public static final double kElevatorMotorGearRatio = 1.0 / 15.0;
        public static final double kBeltPullMeters = kElevatorMotorGearRatio * Math.PI * kPulleyDiameterMeter;
        public static final double kElevatorEncoderRot2Meters = 2 * kBeltPullMeters;
        public static final double kElevatorHeightLimit = 0.0;

        public static final SparkMaxConfig kElevatorMotorConfig = new SparkMaxConfig();
        public static final double kMaxMotorVoltage = 11; // 
        public static final boolean kClampBatteryVoltageToMaxVoltage = true;

        
        // FEEDFORWARD CONSTANTS
        public static final double kS = 0.0;
        public static final double kG = 0.021000;
        public static final double kV = 0.642140;
        public static final double kA = 0.0;

        public static final double kMaxVelocity = 1.25;
        public static final double kMaxAcceleration = 0.85;

        // PID CONSTANTS
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // LEVELS
        public static final double kFirstLevel = 0.0;
        public static final double kSecondLevel = Units.inchesToMeters(12.49 + 4); //12.49
        public static final double kThirdLevel = Units.inchesToMeters(28.33 + 4); //28.33
        public static final double kFourthLevel = Units.inchesToMeters(53.36); //54.36
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7.0);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kMaxMotorRPM = 6380.0;
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

        public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(25);
        // Distance between right and left wheels
        public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(25);
        // Distance between front and back wheels
        public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kDriveKinematics = new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
            new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

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
    
        public static final int kGyroPort = 0;

        public static final int kFrontLeftDriveMotorPort = 8; //mod0
        public static final int kBackLeftDriveMotorPort = 2;//mod1
        public static final int kFrontRightDriveMotorPort = 4; //mod2
        public static final int kBackRightDriveMotorPort = 6;//mod3

        public static final int kFrontLeftTurningMotorPort = 7;//mod0
        public static final int kBackLeftTurningMotorPort = 1;//mod1
        public static final int kFrontRightTurningMotorPort = 3; //mod2
        public static final int kBackRightTurningMotorPort = 5;//mod3

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.3; 
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.2; 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3; 
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = (ModuleConstants.kMaxMotorRPM / (1 / ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI)/ 60;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = (kPhysicalMaxSpeedMetersPerSecond / Units.inchesToMeters(35)) * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.6875;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.2;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kElevatorJoystickPort = 1;

        public static final int kDriverYAxis = 0;
        public static final int kDriverXAxis = 1;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
}

public static final class FieldConstants
{
    // point cords: 0: blue, 1:red and then within that 0:right 1:left, and then within that 0: x 1: y 2: heading
    // point cords: 0: blue, 1:red, and then within that 0: x 1: y 2: heading
    public static final double[][][][] kPointCords = {{{{3.896,5.208,300},{4.186,5.359,300}},{{5.262,5.177,240},{3.960,5.006,240}},{{5.869,3.960,180},{5.866,3.675,180}},
                                                    {{5.111,2.797,120},{4.823,2.643,120}},{{3.776,2.858,60},{3.485,3.029,60}},{{3.169,4.118,0},{5.208,4.408,0}}},

                                                    {{{12.007,4.986,60},{12.320,5.153,60}},{{13.410,5.382,120},{13.670,5.231,120}},{{14.421,4.433,180},{14.421,4.136,180}},
                                                    {{13.722,2.822,240},{13.441,2.687,240}},{{12.353,2.883,300},{12.098,3.029,300}},{{11.710,4.121,0},{11.731,4.371,0}}}};

}}