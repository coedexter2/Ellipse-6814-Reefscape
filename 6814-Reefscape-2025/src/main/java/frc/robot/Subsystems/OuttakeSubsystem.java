package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {

    private final SparkMax m_Out;
    private final DigitalInput m_Beam;
    


    public OuttakeSubsystem()  {
        
        m_Out = new SparkMax(OuttakeConstants.kOuttakeMotorPort, MotorType.kBrushless);
        m_Beam = new DigitalInput(OuttakeConstants.kBeamBreakPort);

        OuttakeConstants.kOuttakeMotorConfig.idleMode(IdleMode.kBrake);
        m_Out.configure(OuttakeConstants.kOuttakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Game Piece", isBroken());
    }

    public boolean isBroken () {
        return !m_Beam.get();
    }

    public void setMotor (double speed) {
        m_Out.set(speed);
    }

}


