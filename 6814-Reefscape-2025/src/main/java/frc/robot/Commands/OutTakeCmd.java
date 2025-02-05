package frc.robot.Commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.OuttakeSubsystem;

public class OutTakeCmd extends Command {
    private final OuttakeSubsystem OuttakeSubsystem;
    private final double speed;

    public OutTakeCmd(OuttakeSubsystem outtakeSubsystem, double speed) 
    {
        this.speed = speed;
        OuttakeSubsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute() 
    {
        OuttakeSubsystem.setMotor(0);
    }

    @Override
    public void end(boolean interrupted)
    {
        OuttakeSubsystem.setMotor(0);
    }
    

    @Override
    public boolean isFinished() 
    {
        return OuttakeSubsystem.isBroken();
    }
}