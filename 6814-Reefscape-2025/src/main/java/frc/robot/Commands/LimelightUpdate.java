package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

public class LimelightUpdate extends Command{

    private final SwerveSubsystem m_SwerveSubsystem;

    public LimelightHelpers.PoseEstimate mt2;
    
    public LimelightUpdate(SwerveSubsystem subsystem) {

        m_SwerveSubsystem = subsystem;

    }

    public void poseEstimatorLimelightUpdate () {
        LimelightHelpers.SetRobotOrientation("", m_SwerveSubsystem.getRotation2d().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        
        if (mt2 != null && mt2.tagCount > 0) {
            m_SwerveSubsystem.visionUpdate(mt2.pose);
            SmartDashboard.putString("Limelight Pose", mt2.pose.toString());
        }
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        poseEstimatorLimelightUpdate();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}