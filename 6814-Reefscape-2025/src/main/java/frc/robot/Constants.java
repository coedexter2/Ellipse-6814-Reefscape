package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Set;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class OuttakeConstants {

        public static final int kOuttakeMotorPort = 21;
        public static final int kBeamBreakPort = 6;
        public static final int kBackBeamBreakPort = 7;
        public static final double kOuttakeSpeed = -1.0;
        public static final double kIntakeSpeed = -0.3;
        public static final double kOuttakeTimeout = 1;

        public static final double kArmGearRatio = 1.0/31.0;
        public static final double kWheelDiamter = 3;
        public static final double kRot2In = kArmGearRatio * Math.PI * kWheelDiamter;

        public static SparkMaxConfig kOuttakeMotorConfig = new SparkMaxConfig();
        public static final double kClearencePos = 0;
        public static final double kScorePos = 1.5;
    
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

        public static final int kElevatorMotorPort = 20;
        public static final int kElevatorLimitSwitchPort = 9;

        public static final double kPulleyDiameterMeter = Units.inchesToMeters(1.504);
        public static final double kElevatorMotorGearRatio = 1.0 / 15.0;
        public static final double kBeltPullMeters = kElevatorMotorGearRatio * Math.PI * kPulleyDiameterMeter;
        public static final double kElevatorEncoderRot2Meters = 3 * kBeltPullMeters;
        public static final double kElevatorHeightLimit = 0.0;

        public static final SparkMaxConfig kElevatorMotorConfig = new SparkMaxConfig();
        public static final double kMaxMotorVoltage = 11; // 
        public static final boolean kClampBatteryVoltageToMaxVoltage = true;

        
        // FEEDFORWARD CONSTANTS
        public static final double kS = 0.0;
        public static final double kG = 0.0367;
        public static final double kV = 0.470000;
        public static final double kA = 0.0;

        public static final double kMaxVelocity = 0.3;
        public static final double kMaxAcceleration = 0.3;

        // PID CONSTANTS
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // LEVELS
        public static final double kFirstLevel = 0.0;
        // public static final double kSecondLevel = Units.inchesToMeters(12.49 + 4); //12.49
        // public static final double kThirdLevel = Units.inchesToMeters(28.33 + 4); //28.33
        // public static final double kFourthLevel = Units.inchesToMeters(53.36); //54.36

        public static final double kSecondLevel = Units.inchesToMeters(31); // if 31 is right, 16 more will get l3, 40+31 will get l4
        public static final double kThirdLevel = Units.inchesToMeters(31+16); // 
        public static final double kFourthLevel = Units.inchesToMeters(40+31-5); //54.36

        public static final double kSourceIntake = Units.inchesToMeters(16);
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

        public static final int kFrontLeftDriveMotorPort = 5; //mod0
        public static final int kBackLeftDriveMotorPort = 1;//mod1
        public static final int kFrontRightDriveMotorPort = 7; //mod2
        public static final int kBackRightDriveMotorPort = 3;//mod3

        public static final int kFrontLeftTurningMotorPort = 6;//mod0
        public static final int kBackLeftTurningMotorPort = 2;//mod1
        public static final int kFrontRightTurningMotorPort = 8; //mod2
        public static final int kBackRightTurningMotorPort = 4;//mod3

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
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.14; 
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.90; 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.6; 
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.33; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = (ModuleConstants.kMaxMotorRPM / (1 / ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI)/ 60;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = (kPhysicalMaxSpeedMetersPerSecond / Units.inchesToMeters(35)) * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9; //* 0.6875 * 1.2
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.2;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kP = 0.8;
        public static final double kI = 0.2;
        public static final double kD = 0.03;
        public static final double kAutoAimMaxAngularSpeed = kTeleDriveMaxAngularSpeedRadiansPerSecond * 0.5;

        public static final int[] kRedStationTags = {2,1};
        public static final int[] kBlueStationTags =  {13,12};

        public static final Translation2d kBlueReefPos = new Translation2d(4.490, 4.020);
        public static final Translation2d kRedReefPos = new Translation2d(13.056, 4.020);
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

public static final class AutoAlignConstants
{
    public static final double kLeftReefOffset = Units.inchesToMeters(-10.25);
    public static final double kRightReefOffset = Units.inchesToMeters(4.25);
    
    public static final int[] kBlueReefTags = { 17, 18, 19, 20, 21, 22 };
    public static final int[] kRedReefTags =  {  6,  7,  8,  9, 10, 11 };

    public static final Set<Integer> kFlippedDirectionTags = Set.of(17, 18, 19, 6, 7, 8); // To make sure the bot goes to the drivers' right when they press right

    // Size of bot in Y Axis (including bumpers) see diagram
    public static final double kBotYSize = Units.inchesToMeters(36); //TODO: Get the correct botysize

    /*
     *   The ^ is the shooter
     * ----------- 
     * |    ^    |
     * |    ^    |  < This side is kBotYSize
     * |         |
     * -----------
     */
}

public static final class ArmConstants
{
    public static final int kArmMotorPort = 9; // fix this 

    public static final double kArmEncoderRot2Meters = 0; // I'll fix this 

    //feedforward
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    //motion profiler
    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;

    // PID 
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    //positions 
    public static final double kDownPose = 0;
    public static final double kUpPose = 0;
    public static final double kL1Pose = 0;
    public static final double kIntakePose = 0;
}

public static final class GroundIntakeConstants
{
    public static final int kIntakeMotorPort = 30; // fix this 
    public static final int kBeambreakPort = 0; // fix this 

    public static final double kGroundIntakeSpeed = 0;
    public static final double kGroundOuttakeSpeed = 0;

    public static final double kOuttakeTimeout = 1;

    
    public static final double kArmEncoderRot2Meters = 0; //FIX THIS!!!!!

    public static final double kL1Pose = 0; //fix this
}


}