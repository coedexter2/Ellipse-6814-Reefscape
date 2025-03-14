package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    int allianceSide;
    int direction;

    public AutoAlign(SwerveSubsystem subsystem, int direction) 
    {
        m_SwerveSubsystem = subsystem;
        this.direction = direction;
        var alliance = DriverStation.getAlliance().get();

     if (alliance == Alliance.Red)
     {
        allianceSide = 1;
     }
     else
     {
        allianceSide = 0;
     }
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        SmartDashboard.putBoolean("isaligned", false);
        
        double x,y,degrees;

        x = Constants.FieldConstants.kPointCords[allianceSide][direction][getSetPoint()][0];
        y = Constants.FieldConstants.kPointCords[allianceSide][direction][getSetPoint()][1];
        degrees = Constants.FieldConstants.kPointCords[allianceSide][direction][getSetPoint()][2];

        Pose2d startPose = m_SwerveSubsystem.getPose();
        Pose2d finalPose =  new Pose2d(x, y, Rotation2d.fromDegrees(Math.atan2(m_SwerveSubsystem.getPose().getX()-x,m_SwerveSubsystem.getPose().getY()-y)));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, finalPose);

        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(degrees)) 
        );
        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_SwerveSubsystem.stopModules();
        SmartDashboard.putBoolean("isaligned", true);
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;
    }

    public int getSetPoint()
    {
        double minDistance = 2000000000;
        int point = -1;
            for(int i = 0;i < Constants.FieldConstants.kPointCords[allianceSide].length;i++)
            {
                double distance = getDistance(Constants.FieldConstants.kPointCords[allianceSide][direction][i][0], Constants.FieldConstants.kPointCords[allianceSide][direction][i][1]);
                if(distance<minDistance)
                {
                    minDistance = distance;
                    point = i;
                }
            }

        return point;
    }

    public double getDistance(double x, double y)
    {
        return (Math.pow((x - m_SwerveSubsystem.getPose().getX()), 2)) - (Math.pow((y + m_SwerveSubsystem.getPose().getY()), 2));
    }
}