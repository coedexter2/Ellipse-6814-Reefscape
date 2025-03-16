package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

@Logged
public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax m_climbMotor = new SparkMax(ClimbConstants.kClimbMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_climbEncoder = m_climbMotor.getEncoder();
    private final DigitalInput m_Limit;
   
    public ClimbSubsystem() {
        ClimbConstants.kClimbMotorConfig.idleMode(IdleMode.kCoast);
        m_climbMotor.configure(ClimbConstants.kClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Limit = new DigitalInput(ClimbConstants.kLimitSwitchPort);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position", getEncoderPosition());
        SmartDashboard.putBoolean("Climb Limit Switch", isHome());
    }

    public void setSpeed(double speed) {

        m_climbMotor.set(speed);
    }


    public void resetEncoder() {
        
        m_climbEncoder.setPosition(0);
    }


    public double getEncoderPosition() {

        return m_climbEncoder.getPosition() * ClimbConstants.kClimbEncoderRotToRadians;

    }
    
    public void setIdleMode(IdleMode idleMode)
    {
        ClimbConstants.kClimbMotorConfig.idleMode(idleMode);
        m_climbMotor.configure(ClimbConstants.kClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    public boolean isHome()
    {
        return !m_Limit.get();
    }
}
