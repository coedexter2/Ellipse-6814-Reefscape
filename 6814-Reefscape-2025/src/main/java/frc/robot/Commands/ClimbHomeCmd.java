package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;

public class ClimbHomeCmd extends Command {
    private final ClimbSubsystem m_ClimbSubsystem;
    private final double speed;

    public ClimbHomeCmd(ClimbSubsystem subsystem, double speed) 
    {
        this.speed = speed;
        m_ClimbSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        m_ClimbSubsystem.setSpeed(speed);
        SmartDashboard.putBoolean("climb limit swithc", m_ClimbSubsystem.isHome());
    }

    @Override
    public void end(boolean interrupted)
    {
        m_ClimbSubsystem.setSpeed(0);
        m_ClimbSubsystem.resetEncoder();
    }
    

    @Override
    public boolean isFinished() 
    {
        return m_ClimbSubsystem.isHome();   
    }
}