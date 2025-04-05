package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class OuttakeCmd extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final double speed;

    public OuttakeCmd(OuttakeSubsystem subsystem, double speed) 
    {
        this.speed = speed;
        m_OuttakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        SmartDashboard.putNumber("WHAT THE FRICK", speed);
        m_OuttakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_OuttakeSubsystem.setMotor(0);
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;   
    }
}