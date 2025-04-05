package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class IntakeClearenceCmd extends Command {
    private final OuttakeSubsystem m_OuttakeSubsystem;
    private final double speed;
    private final double setpoint;

    public IntakeClearenceCmd(OuttakeSubsystem subsystem, double speed, double setpoint) 
    {
        this.speed = speed;
        this.setpoint = setpoint;
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
        m_OuttakeSubsystem.resetEncoder();
    }


    @Override
    public boolean isFinished() 
    {
        return m_OuttakeSubsystem.getEncoder() >= setpoint;
    }
}