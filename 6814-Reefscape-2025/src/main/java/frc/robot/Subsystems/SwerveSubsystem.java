package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


@Logged
public class SwerveSubsystem extends SubsystemBase {
    private final static SwerveModule frontLeft = new SwerveModule(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    
        private final static SwerveModule frontRight = new SwerveModule(
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    
        private final static SwerveModule backLeft = new SwerveModule(
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    
        private final static SwerveModule backRight = new SwerveModule(
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
        
        private final SwerveModule[] modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
    
        private final Pigeon2 gyro = new Pigeon2(0);
    
        private Field2d field = new Field2d();
        

        private final SwerveDrivePoseEstimator m_poseEstimator = 
            new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    backLeft.getPosition(),
                    frontRight.getPosition(),
                    backRight.getPosition()
                }, new Pose2d(0, 0, new Rotation2d()));
            
                public SwerveSubsystem() {
            
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

                    try {

                        RobotConfig config = RobotConfig.fromGUISettings();
            
                        AutoBuilder.configure(
                        this::getPose, 
                        this::resetOdometry, 
                        this::getSpeeds, 
                        this::driveRobotRelative, 
                            new PPHolonomicDriveController(
                            AutoConstants.translationConstants,
                            AutoConstants.rotationConstants
                        ),
                        config,
                        () -> {
                            // Boolean supplier that controls when the path will be mirrored for the red alliance
                            // This will flip the path being followed to the red side of the field.
                            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                return alliance.get() == DriverStation.Alliance.Red;
                            }
                            return false;
                    },
                    this
                  );
                }catch(Exception e){
                  DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
                }
            
                // Set up custom logging to add the current path to a field 2d widget
                PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
            
                SmartDashboard.putData("Field", field);
              }

                    
            
                public void zeroHeading() {
                    gyro.reset();
                }
            
                public double getHeading() {
                    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
                }
            
                public Rotation2d getRotation2d() {
                    return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void updatePoseOnField(String name, Pose2d pose) {
        field.getObject(name).setPose(pose.getX(), pose.getY(), pose.getRotation());
      }


    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        },
        pose);  
    }

    public void visionUpdate(Pose2d pose) {
        m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());

    }

    public  ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        // Thank you michael jansen
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }



    @Override
    public void periodic() {
        m_poseEstimator.update(getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
            
        });

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().toString());

        frontLeft.writeValues();
        frontRight.writeValues();
        backLeft.writeValues();
        backRight.writeValues();

        updatePoseOnField("Field", getPose());
        
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}