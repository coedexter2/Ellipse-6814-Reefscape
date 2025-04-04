package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GroundIntakeSubsystem;

public class SecureCoral extends Command {
    private final GroundIntakeSubsystem m_GroundIntakeSubsystem;
    private final double speed;
    private final double setpoint;

    public SecureCoral(GroundIntakeSubsystem subsystem, double speed, double setpoint) 
    {
        this.speed = speed;
        this.setpoint = setpoint;
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
        m_GroundIntakeSubsystem.resetEncoder();
    }


    @Override
    public boolean isFinished() 
    {
        return m_GroundIntakeSubsystem.getEncoder() == setpoint;
    }
}