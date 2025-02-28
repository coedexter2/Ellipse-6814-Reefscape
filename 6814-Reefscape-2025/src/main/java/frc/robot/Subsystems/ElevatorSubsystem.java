package frc.robot.Subsystems;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax m_elevator;
    private final RelativeEncoder m_encoder;

    public ElevatorSubsystem() {

        m_elevator = new SparkMax(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
        m_elevator.configure(ElevatorConstants.kElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_elevator.getEncoder();

    }

    public void setMotor(double speed) {

        m_elevator.set(speed);

    }

    public void resetEncoder() {

        m_encoder.setPosition(0);

    }

    public double getEncoderPosition() {
        
        return m_encoder.getPosition() * ElevatorConstants.kElevatorEncoderRot2Meters;
    }

    public double getEncoderVelocity() {

        return m_encoder.getVelocity();
    }
}