package frc.robot.Commands;
import java.lang.reflect.Array;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;

    public AutoAlign(SwerveSubsystem subsystem) 
    {
        m_SwerveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        m_SwerveSubsystem.getPose();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_SwerveSubsystem.stopModules();
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;
    }

    public String getSetPoint(list point[list axis[]])
    {
        for(i=0;i<Length())
        return "ababab";
    }

    public double getDistance(double x, double y)
    {
        return (Math.pow((x - m_SwerveSubsystem.getPose().getX()), 2)) - (Math.pow((y + m_SwerveSubsystem.getPose().getY()), 2));
    }
}