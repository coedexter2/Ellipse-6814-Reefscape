package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class IntakeClearenceCmd extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final double speed;
    private final double distance;
    private double initial = 0;

    public IntakeClearenceCmd(OuttakeSubsystem subsystem, double speed, double distance) 
    {
        this.speed = speed;
        this.distance = distance;
        m_OuttakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        initial = m_OuttakeSubsystem.getEncoder();
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


    @Override
    public boolean isFinished() 
    {
        System.out.println("Just finished going to " + distance + "!");
        return m_OuttakeSubsystem.getEncoder() <= initial - distance || m_OuttakeSubsystem.getEncoder() >= initial + distance;
    }
}