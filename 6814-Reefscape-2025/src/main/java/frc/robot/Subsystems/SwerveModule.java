package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.hardware.TalonFX;;
@Logged
public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    @SuppressWarnings( "removal" )
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        angle *= (Math.PI * 2);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {   
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRad()/Constants.ModuleConstants.kTurningEncoderRot2Rad);
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }   
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void writeValues(){  
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", 
        getState().toString() + " abs encoder rad " + getAbsoluteEncoderRad());
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] drive pos", getPosition().toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}