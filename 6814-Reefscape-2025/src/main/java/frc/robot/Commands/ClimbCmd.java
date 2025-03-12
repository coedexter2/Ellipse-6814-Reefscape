package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbCmd extends Command {
    private ClimbSubsystem m_ClimbSubsystem;
    private final double speed;
    private double setpoint;
    private final BangBangController m_BangBangController = new BangBangController();


    public ClimbCmd(ClimbSubsystem subsystem, double setpoint, double speed) {
        this.speed = speed;
        this.setpoint = setpoint;
        m_ClimbSubsystem = subsystem;
    }


    @Override
    public void execute() {
        m_ClimbSubsystem.setSpeed(m_BangBangController.calculate(m_ClimbSubsystem.getEncoderPosition(), setpoint) * speed);
        SmartDashboard.putNumber("Climb Output", m_BangBangController.calculate(m_ClimbSubsystem.getEncoderPosition(), setpoint) * speed);
    }


    @Override
    public void end(boolean interrupted) {
        m_ClimbSubsystem.setSpeed(0);
    }
    

    @Override
    public boolean isFinished() {
        return m_BangBangController.calculate(m_ClimbSubsystem.getEncoderPosition(), setpoint) == 0.0;
    }
}

