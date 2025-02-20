package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        var alliance = DriverStation.getAlliance().get();
        double x,y,degrees;
        if(alliance == Alliance.Red)
        {
            x = Constants.FeildConstants.kRedMidpointCords[getSetPoint()][0];
            y = Constants.FeildConstants.kRedMidpointCords[getSetPoint()][1];
            degrees = Constants.FeildConstants.kRedMidpointCords[getSetPoint()][2];
        }
        else
        {
            x = Constants.FeildConstants.kBlueMidpointCords[getSetPoint()][0];
            y = Constants.FeildConstants.kBlueMidpointCords[getSetPoint()][1];
            degrees = Constants.FeildConstants.kRedMidpointCords[getSetPoint()][2];
        }
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(x, y, Rotation2d.fromDegrees(Math.atan2(m_SwerveSubsystem.getPose().getX()-x,m_SwerveSubsystem.getPose().getY()-y)))
        );

        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(degrees)) 
        );
        path.preventFlipping = true;
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