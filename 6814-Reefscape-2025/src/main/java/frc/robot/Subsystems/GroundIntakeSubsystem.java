package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.OuttakeConstants;
@Logged
public class GroundIntakeSubsystem extends SubsystemBase {

    private final SparkMax m_Out;
    private final RelativeEncoder m_Encoder;
    


    public GroundIntakeSubsystem()  {
        
        m_Out = new SparkMax(GroundIntakeConstants.kArmMotorPort, MotorType.kBrushless);
        m_Encoder = m_Out.getEncoder();
        OuttakeConstants.kOuttakeMotorConfig.idleMode(IdleMode.kBrake);

    }

    public void setMotor (double speed) {
        m_Out.set(speed);
    }

    public double getEncoder ()
    {
        return m_Encoder.getPosition() * GroundIntakeConstants.kArmEncoderRot2Meters;
    }

    public void resetEncoder(){
        m_Encoder.setPosition(0);
    }

}


