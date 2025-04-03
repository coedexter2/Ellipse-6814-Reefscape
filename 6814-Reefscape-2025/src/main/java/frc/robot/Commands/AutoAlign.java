package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private Command pathplannerDriveCommand;

    public AutoAlign(SwerveSubsystem subsystem, ReefAlignment alignment) 
    {
        m_SwerveSubsystem = subsystem;
        this.alignment = alignment;

        // addRequirements(subsystem);
    }

    @Override
    public void initialize() 
    {
        SmartDashboard.putBoolean("isaligned", false);
        
        int closestAprilTagID = getClosestAprilTag();
        Pose2d closestAprilTagPose = fieldTags.getTagPose(closestAprilTagID).get().toPose2d();
        double tagRotation = closestAprilTagPose.getRotation().getRadians() + Math.PI;
        double x, y;

        // Flip if the AprilTag is on the side of the reef closest to the driver so that the drivers' right is the robot's right
        ReefAlignment trueAlignment = AutoAlignConstants.kFlippedDirectionTags.contains(closestAprilTagID) ? alignment.opposite() : alignment;

        if(trueAlignment == ReefAlignment.LEFT)
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
        Pose2d finalPose = new Pose2d(x, y, closestAprilTagPose.getRotation().plus(Rotation2d.k180deg));

        double distanceBetweenPoses = Math.sqrt(Math.pow(finalPose.getX() - startPose.getX(), 2) +
                                                Math.pow(finalPose.getY() - startPose.getY(), 2));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, finalPose);

        PathConstraints constraints = new PathConstraints(2, 1.4, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        List<RotationTarget> rotationTargets = new ArrayList<>(); // Empty list
        List<PointTowardsZone> pointTowardsZones = new ArrayList<>();
        List<EventMarker> eventMarkers = new ArrayList<>();

        List<ConstraintsZone> constraintsZones = Arrays.asList(new ConstraintsZone(Math.min(1.0, 1.0 - (0.5 / distanceBetweenPoses)), 1.0, new PathConstraints(0.6, 5, 2 * Math.PI, 4 * Math.PI)));

        PathPlannerPath path = new PathPlannerPath( 
                waypoints,
                rotationTargets,
                pointTowardsZones,
                constraintsZones,
                eventMarkers,
                constraints,
                null,
                //new GoalEndState(0.0, Rotation2d.fromDegrees(degrees))
                new GoalEndState(0.0, Rotation2d.fromRadians(tagRotation)), //TODO: if bot is aligning the wrong way chang this rotation
                false
        );
        path.preventFlipping = true;

        pathplannerDriveCommand = AutoBuilder.followPath(path).raceWith(new LimelightUpdate(m_SwerveSubsystem));
        pathplannerDriveCommand.schedule();

        SmartDashboard.putString("TargetPose", finalPose.toString());
    }

    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {
        // m_SwerveSubsystem.stopModules();
        SmartDashboard.putNumber("I hate cooper", Timer.getFPGATimestamp());
        pathplannerDriveCommand.cancel();
        SmartDashboard.putBoolean("isaligned", true);
    }
    

    @Override
    public boolean isFinished() 
    {
        return pathplannerDriveCommand.isFinished();
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