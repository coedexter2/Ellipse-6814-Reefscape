package frc.robot.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class IntakeCmd extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final double speed;
    private boolean delayFinished = false;
    private boolean beamBroken = false;
    private double brokenDistance;

    public IntakeCmd(OuttakeSubsystem subsystem, double speed) 
    {
        this.speed = speed;
        m_OuttakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        delayFinished = false;
        beamBroken = false;
    }

    @Override
    public void execute() 
    {
        m_OuttakeSubsystem.setMotor(speed);

        SmartDashboard.putNumber("PLEASE WORK PLEASE", Timer.getFPGATimestamp());
        if(m_OuttakeSubsystem.isBroken() && !beamBroken) { beamBroken = true; brokenDistance = m_OuttakeSubsystem.getEncoder(); }
        SmartDashboard.putNumber("brokenDistance", brokenDistance);
        SmartDashboard.putNumber("Outtake Encoder", m_OuttakeSubsystem.getEncoder());
        if(beamBroken && m_OuttakeSubsystem.getEncoder() <= brokenDistance - 2)
        {
            delayFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_OuttakeSubsystem.setMotor(0);
        SmartDashboard.putNumber("thingtestaoisdufjasdfasjio", Timer.getFPGATimestamp());
    }


    @Override
    public boolean isFinished() 
    {
        
        return delayFinished;
    }
}