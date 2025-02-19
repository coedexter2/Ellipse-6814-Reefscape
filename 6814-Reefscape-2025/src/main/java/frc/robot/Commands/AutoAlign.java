package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        // made the get setpoint thing so it will return the num 0-7 of the closest midpoint on feild
        // now its just a matter of adding what will drive it to that set point
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

    public int getSetPoint()
    {
        var alliance = DriverStation.getAlliance().get();
        double minDistance = 2000000000;
        int point = -1;
        if(alliance == Alliance.Red)
        {
            for(int i = 0;i < Constants.FeildConstants.kRedMidpointCords.length;i++)
            {
                double distance = getDistance(Constants.FeildConstants.kRedMidpointCords[i][0], Constants.FeildConstants.kRedMidpointCords[i][1]);
                if(distance<minDistance)
                {
                    minDistance = distance;
                    point = i;
                }
            }
        }

        else
        {
            for(int i = 0;i < Constants.FeildConstants.kRedMidpointCords.length;i++)
            {
                double distance = getDistance(Constants.FeildConstants.kBlueMidpointCords[i][0], Constants.FeildConstants.kBlueMidpointCords[i][1]);
                if(distance<minDistance)
                {
                    minDistance = distance;
                    point = i;
                }
            }
        }

        return point;
    }

    public double getDistance(double x, double y)
    {
        return (Math.pow((x - m_SwerveSubsystem.getPose().getX()), 2)) - (Math.pow((y + m_SwerveSubsystem.getPose().getY()), 2));
    }
}