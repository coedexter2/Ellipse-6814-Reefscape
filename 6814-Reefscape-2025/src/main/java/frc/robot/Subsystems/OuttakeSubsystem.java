package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.OuttakeCmd;
import frc.robot.Constants.OuttakeConstants;
@Logged
public class OuttakeSubsystem extends SubsystemBase {

    private final SparkMax m_Out;
    private final DigitalInput m_Beam;
    private final DigitalInput m_BackBeam;
    private final RelativeEncoder m_Encoder;


    public OuttakeSubsystem()  {
        
        m_Out = new SparkMax(OuttakeConstants.kOuttakeMotorPort, MotorType.kBrushless);
        m_Beam = new DigitalInput(OuttakeConstants.kBeamBreakPort);
        m_BackBeam = new DigitalInput(OuttakeConstants.kBackBeamBreakPort);
        m_Encoder = m_Out.getEncoder();

        OuttakeConstants.kOuttakeMotorConfig.idleMode(IdleMode.kBrake);
        m_Out.configure(OuttakeConstants.kOuttakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Game Piece", isBroken());
        SmartDashboard.putBoolean("back", isBackBroken());
    }

    public boolean isBroken () {
        return !m_Beam.get();
    }

    public boolean isBackBroken () {
        return !m_BackBeam.get();
    }


    public void setMotor (double speed) {
        m_Out.set(speed);
    }

    public void resetEncoder(){
        m_Encoder.setPosition(0);
    }
    public double getEncoder (){
        return m_Encoder.getPosition() * OuttakeConstants.kRot2In;
    }

}


