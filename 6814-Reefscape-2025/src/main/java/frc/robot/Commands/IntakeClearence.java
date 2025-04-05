package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

public class IntakeClearence extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double speed;

    public IntakeClearence(OuttakeSubsystem subsystem,ElevatorSubsystem elevatorSubsystem ,double speed) 
    {
        m_ElevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        m_OuttakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        if(m_ElevatorSubsystem.getLimitSwitch())
        {
            m_OuttakeSubsystem.setMotor(speed);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_OuttakeSubsystem.setMotor(0);
    }


    @Override
    public boolean isFinished() 
    {
        return m_OuttakeSubsystem.isBackBroken();
    }
}