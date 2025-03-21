package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class SourceLockCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, slowSupplier;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter, slowX, slowY;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final PIDController controller = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        private final AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    double speedModifer;
    public SourceLockCmd(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Double> slowSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.m_ElevatorSubsystem = elevatorSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.slowSupplier = slowSupplier;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.slowX = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond * 1.5);
        this.slowY = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond * 1.5);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        speedModifer = 1;
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs and angle lock instead of turning 
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        Pose2d closestAprilTagPose = fieldTags.getTagPose(getClosestAprilTag()).get().toPose2d();
        double Rotation = closestAprilTagPose.getRotation().getRadians();
        controller.setPID(SmartDashboard.getNumber("kayp", 0.0), 0.0, 0.0);
        double turningSpeed = controller.calculate(Units.degreesToRadians(swerveSubsystem.getHeading()), Rotation);
        SmartDashboard.putNumber("Target Angle Aimlock", Units.radiansToDegrees(Rotation));
        SmartDashboard.putNumber("aimlock output", turningSpeed);

        if (slowSupplier.get() > 0.5) {
            speedModifer = 0.5;

        }
        else {
            speedModifer = 1;
        }


        if (m_ElevatorSubsystem.getEncoderPosition() <= ElevatorConstants.kThirdLevel) {
            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = speedModifer * Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        }
        else {
            xSpeed = speedModifer * 0.5 * Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = speedModifer * 0.5 * Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = speedModifer * 0.5 * Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        }
        
        // 3. Make the driving smoother
        if (slowSupplier.get() < 0.5) {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        }
        else {
            xSpeed = slowX.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = slowY.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        }


        var alliance = DriverStation.getAlliance();
        var invert = 1;
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                invert = -1;
            }
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * invert * speedModifer, ySpeed * invert * speedModifer, turningSpeed, swerveSubsystem.getRotation2d());
            } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed * speedModifer, ySpeed * speedModifer, turningSpeed);
        }
        
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public int getClosestAprilTag()
    {
        int[] myStation;
        if(DriverStation.getAlliance().get() == Alliance.Red)
        {
            myStation = Constants.DriveConstants.kRedStationTags;
        }
        else
        {
            myStation = Constants.DriveConstants.kBlueStationTags;
        }
        
        int closestTag = -1;
        double minDistance = Double.POSITIVE_INFINITY;
        for(int tagID : myStation)
        {
            Pose2d tagPosition = fieldTags.getTagPose(tagID).get().toPose2d();
            double distance = getDistanceBetweenBotAndPoint(tagPosition.getX(), tagPosition.getY());
            if(distance < minDistance)
            {
                closestTag = tagID;
                minDistance = distance;
            }
        }

        return closestTag;
    }

    public double getDistanceBetweenBotAndPoint(double x, double y)
    {
        return (Math.pow((x - swerveSubsystem.getPose().getX()), 2)) + (Math.pow((y - swerveSubsystem.getPose().getY()), 2));
    }
}