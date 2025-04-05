package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCmd extends Command {

    private ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kS, 
                                                                ArmConstants.kG, 
                                                                ArmConstants.kV,
                                                                ArmConstants.kA);
    private TrapezoidProfile m_motionProfiler = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, 
                                                                                                      ArmConstants.kMaxAcceleration));
    private TrapezoidProfile.State setpoint;
    private final PIDController m_PID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

    private final ArmSubsystem m_ArmSubsystem;
    private final double position;

    public ArmCmd(ArmSubsystem subsystem, double position)
    {
        m_ArmSubsystem = subsystem;
        this.position = position;
        this.setpoint = new TrapezoidProfile.State(m_ArmSubsystem.getEncoderPosition(), 
                                                   0.0); 
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_feedforward = new ArmFeedforward(SmartDashboard.getNumber("ArmKs", 0.0),
                                                SmartDashboard.getNumber("ArmKg", 0.0),
                                                SmartDashboard.getNumber("ArmKv", 0.0),
                                                SmartDashboard.getNumber("ArmKa", 0.0));

        m_PID.setPID(SmartDashboard.getNumber("ArmKp", 0.0),
                     SmartDashboard.getNumber("ArmKi", 0.0),
                     SmartDashboard.getNumber("ArmKd", 0.0));

        SmartDashboard.putNumber("goal", position);
    }

    @Override
    public void execute() 
    {
        TrapezoidProfile.State nextSetpoint = m_motionProfiler.calculate(0.02, 
                                                                         setpoint, 
                                                                         new TrapezoidProfile.State(position, 0.0));
    
        double feedforwardOutput = m_feedforward.calculate(nextSetpoint.position, nextSetpoint.velocity);
        double pidOutput = m_PID.calculate(nextSetpoint.position, nextSetpoint.position);

        double outputs = feedforwardOutput + pidOutput;
        this.setpoint = nextSetpoint;
        
        m_ArmSubsystem.setMotor(outputs);

        SmartDashboard.putNumber("elevatoroutput", outputs);
        SmartDashboard.putNumber("profiler setpoint", nextSetpoint.position);
        SmartDashboard.putNumber("elevator pos", m_ArmSubsystem.getEncoderPosition());
    }

    @Override
    public void end(boolean interrupted)
    {
        m_ArmSubsystem.setMotor(0);
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
