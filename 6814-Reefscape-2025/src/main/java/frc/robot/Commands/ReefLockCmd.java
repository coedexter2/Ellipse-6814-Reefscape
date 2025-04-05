package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class ReefLockCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, slowSupplier;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter, slowX, slowY;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    double speedModifer;

    private final PIDController reefLockPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

    private Translation2d targetReef;
    public ReefLockCmd(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem,
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
        targetReef = DriverStation.getAlliance().get() == Alliance.Red ? DriveConstants.kRedReefPos : DriveConstants.kBlueReefPos;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        Pose2d swervePose = swerveSubsystem.getPose();
        Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(targetReef.getY() - swervePose.getY(), targetReef.getX() - swervePose.getX()));
        Rotation2d error = swervePose.getRotation().minus(targetRotation);

        double turningSpeed = reefLockPID.calculate(error.getRadians(), 0);
        
        if (slowSupplier.get() > 0.5) {
            speedModifer = 0.25;

        }
        else {
            speedModifer = 1;
        }

        if (m_ElevatorSubsystem.getEncoderPosition() <= ElevatorConstants.kThirdLevel) {
            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;        }
        else {
            xSpeed = speedModifer * 1.0 * Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = speedModifer * 1.0 * Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;        }
        
        // 3. Make the driving smoother
        if (slowSupplier.get() < 0.5) {
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        }
        else {
            xSpeed = slowX.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = slowY.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        }

        turningSpeed *= DriveConstants.kAutoAimMaxAngularSpeed;
        turningSpeed = Math.max(Math.min(turningSpeed, DriveConstants.kAutoAimMaxAngularSpeed), -DriveConstants.kAutoAimMaxAngularSpeed);

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
}