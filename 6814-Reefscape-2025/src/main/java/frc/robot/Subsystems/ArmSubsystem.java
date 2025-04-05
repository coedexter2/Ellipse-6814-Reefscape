package frc.robot.Subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class ArmSubsystem extends SubsystemBase{

    private final SparkMax m_arm;
    private final RelativeEncoder m_encoder;

    public ArmSubsystem() {

        m_arm = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
        m_encoder = m_arm.getEncoder();
    }

    public void setMotor(double speed) 
    {
            m_arm.setVoltage(-speed);
    }

    public void resetEncoder()
    {
        m_encoder.setPosition(0);
    }

    public double getEncoderPosition() {
        
        return -m_encoder.getPosition() * ArmConstants.kArmEncoderRot2Meters;
    }

    public double getEncoderVelocity() {

        return -m_encoder.getVelocity() * ArmConstants.kArmEncoderRot2Meters;
    }
}