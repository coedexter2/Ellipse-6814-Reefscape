package frc.robot.Subsystems;

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
    


    public GroundIntakeSubsystem()  {
        
        m_Out = new SparkMax(GroundIntakeConstants.kArmMotorPort, MotorType.kBrushless);

        OuttakeConstants.kOuttakeMotorConfig.idleMode(IdleMode.kBrake);

    }

    public void setMotor (double speed) {
        m_Out.set(speed);
    }

}


