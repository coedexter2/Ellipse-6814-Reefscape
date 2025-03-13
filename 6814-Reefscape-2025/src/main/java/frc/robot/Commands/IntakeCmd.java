package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void initialize() {
        broken = false;
    }

    @Override
    public void execute() 
    {
        m_OuttakeSubsystem.setMotor(speed);

        SmartDashboard.putBoolean("beambreak", m_OuttakeSubsystem.isBroken());
        SmartDashboard.putBoolean("broken", broken);
    }

    public boolean finish() {
        
        if (m_OuttakeSubsystem.isBroken()) {
            
            SmartDashboard.putBoolean("isbrokenbeam", false);
            broken = true;
        }

        if (broken && !m_OuttakeSubsystem.isBroken()){

            SmartDashboard.putBoolean("isbrokenbeam", false);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_OuttakeSubsystem.setMotor(0);
    }


    @Override
    public boolean isFinished() 
    {
        return finish();
    }
}