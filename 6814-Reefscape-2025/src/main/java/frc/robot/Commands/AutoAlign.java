package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.ReefAlignment;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final ReefAlignment alignment;

    public AutoAlign(SwerveSubsystem subsystem, ReefAlignment alignment) 
    {
        m_SwerveSubsystem = subsystem;
        this.alignment = alignment;

        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        SmartDashboard.putBoolean("isaligned", false);
        
        Pose2d closestAprilTagPose = fieldTags.getTagPose(getClosestAprilTag()).get().toPose2d();
        double tagRotation = closestAprilTagPose.getRotation().getRadians() + Math.PI;
        double x, y;

        if(alignment == ReefAlignment.LEFT)
        {
            x = closestAprilTagPose.getX()
                + Math.cos(tagRotation + Math.PI) * (AutoAlignConstants.kBotYSize / 2) // Push bot out of reef
                + Math.cos(tagRotation + Math.PI / 2) * AutoAlignConstants.kLeftReefOffset; // Offset to the left
            y = closestAprilTagPose.getY() 
                + Math.sin(tagRotation + Math.PI) * (AutoAlignConstants.kBotYSize / 2)
                + Math.sin(tagRotation + Math.PI / 2) * AutoAlignConstants.kLeftReefOffset;
        }
        else
        {
            x = closestAprilTagPose.getX()
                + Math.cos(tagRotation + Math.PI) * (AutoAlignConstants.kBotYSize / 2) // Push bot out of reef
                + Math.cos(tagRotation + Math.PI / 2) * AutoAlignConstants.kRightReefOffset; // Offset to the right
            y = closestAprilTagPose.getY() 
                + Math.sin(tagRotation + Math.PI) * (AutoAlignConstants.kBotYSize / 2)
                + Math.sin(tagRotation + Math.PI / 2) * AutoAlignConstants.kRightReefOffset;
        }

        Pose2d swervePose = m_SwerveSubsystem.getPose();
        Rotation2d rotationBetwenSwerveAndTarget = Rotation2d.fromRadians(Math.atan2(m_SwerveSubsystem.getPose().getY()-y, m_SwerveSubsystem.getPose().getX()-x));
        Pose2d startPose = new Pose2d(swervePose.getX(), swervePose.getY(), rotationBetwenSwerveAndTarget); 
        Pose2d finalPose = new Pose2d(x, y, rotationBetwenSwerveAndTarget);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, finalPose);

        PathConstraints constraints = new PathConstraints(1.2, 1.4, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                //new GoalEndState(0.0, Rotation2d.fromDegrees(degrees))
                new GoalEndState(0.0, Rotation2d.fromRadians(tagRotation)) //TODO: if bot is aligning the wrong way chang this rotation 
        );
        path.preventFlipping = true;

        AutoBuilder.followPath(path).raceWith(new LimelightUpdate(m_SwerveSubsystem)).schedule();
    }

    @Override
    public void end(boolean interrupted)
    {
        // m_SwerveSubsystem.stopModules();
        // m_SwerveSubsystem.getCurrentCommand().cancel();
        SmartDashboard.putBoolean("isaligned", true);
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;
    }

    public int getClosestAprilTag()
    {
        int[] myReef;
        if(DriverStation.getAlliance().get() == Alliance.Red) //TODO: When we fix the apriltag fix this please (its reversed alliances)
        {
            myReef = AutoAlignConstants.kRedReefTags;
        }
        else
        {
            myReef = AutoAlignConstants.kBlueReefTags;
        }
        
        int closestTag = 0;
        double minDistance = Double.POSITIVE_INFINITY;
        for(int tagID : myReef)
        {
            Pose2d tagPosition = fieldTags.getTagPose(tagID).get().toPose2d();
            double distance = getDistanceBetweenBotAndPoint(tagPosition.getX(), tagPosition.getY());
            if(distance < minDistance)
            {
                closestTag = tagID;
                minDistance = distance;
            }
        }

        return closestTag;
    }

    public double getDistanceBetweenBotAndPoint(double x, double y)
    {
        return (Math.pow((x - m_SwerveSubsystem.getPose().getX()), 2)) + (Math.pow((y - m_SwerveSubsystem.getPose().getY()), 2));
    }
}