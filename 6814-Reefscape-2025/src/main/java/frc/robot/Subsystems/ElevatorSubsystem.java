package frc.robot.Subsystems;

import frc.robot.Commands.ElevatorCommand;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax m_elevator;
    private final RelativeEncoder m_encoder;
    private final DigitalInput m_limitSwitch;

    public ElevatorSubsystem() {

        m_elevator = new SparkMax(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
        m_elevator.configure(ElevatorConstants.kElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_limitSwitch = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchPort);

        m_encoder = m_elevator.getEncoder();

    }

    /**
     * Automatically limits voltage to ElevatorConstants.kMaxMotorVoltage
     * @param speed Speed -1 to 1
     */
    public void setMotor(double speed) {
        if(ElevatorConstants.kClampBatteryVoltageToMaxVoltage)
        {
            m_elevator.setVoltage(Math.max(Math.min(speed, 1.0), -1.0) * ElevatorConstants.kMaxMotorVoltage);
        }
        else
        {
            m_elevator.setVoltage(speed * ElevatorConstants.kMaxMotorVoltage);
        }
        
        // m_elevator.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator pos", getEncoderPosition());
        getLimitSwitch();

    }

    public void resetEncoder() {

        m_encoder.setPosition(0.05);

    }

    public double getEncoderPosition() {
        
        return m_encoder.getPosition() * ElevatorConstants.kElevatorEncoderRot2Meters;
    }

    public double getEncoderVelocity() {

        return m_encoder.getVelocity() * ElevatorConstants.kElevatorEncoderRot2Meters;
    }

    public boolean getLimitSwitch()
    {
        return !m_limitSwitch.get();
    }
}