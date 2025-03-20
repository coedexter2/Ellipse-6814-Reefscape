package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GroundIntakeSubsystem;

public class LoadCmd extends Command {
    private final GroundIntakeSubsystem m_GroundIntakeSubsystem;
    private final double speed;

    public LoadCmd(GroundIntakeSubsystem subsystem, double speed) 
    {
        this.speed = speed;
        m_GroundIntakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        m_GroundIntakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_GroundIntakeSubsystem.setMotor(0);
    }


    @Override
    public boolean isFinished() 
    {
        return false;
    }
}