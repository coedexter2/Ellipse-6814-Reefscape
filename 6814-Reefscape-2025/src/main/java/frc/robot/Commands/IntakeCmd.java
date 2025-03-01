package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class IntakeCmd extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final double speed;
    private boolean broken;

    public IntakeCmd(OuttakeSubsystem subsystem, double speed) 
    {
        this.speed = speed;
        m_OuttakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        m_OuttakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_OuttakeSubsystem.setMotor(0);
    }

    public boolean finish()
    {
        if (m_OuttakeSubsystem.isBroken() == true)
        {
            broken = true;
        }
        if (broken == true && m_OuttakeSubsystem.isBroken() == false)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    

    @Override
    public boolean isFinished() 
    {
        return finish();
    }
}