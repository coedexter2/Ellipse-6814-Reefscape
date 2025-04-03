package frc.robot.Commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    // ===== Feedforward stuff =====
    private final Timer m_profilerTimer = new Timer();
    
    private TrapezoidProfile m_motionProfiler = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, 
                                                                                                      ElevatorConstants.kMaxAcceleration));
    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, 
                                                                              ElevatorConstants.kG, 
                                                                              ElevatorConstants.kV,
                                                                              ElevatorConstants.kA);
    private TrapezoidProfile.State setpoint;
    private boolean robotIsTryingToDestroyItself = false;
    
    // ===== PID stuff =====
    private final PIDController m_pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double position;

    public ElevatorCommand(ElevatorSubsystem subsystem, double position)
    {
        this.m_elevatorSubsystem = subsystem;
        this.position = position;
        this.setpoint = new TrapezoidProfile.State(m_elevatorSubsystem.getEncoderPosition(), 
                                                   0.0);
        this.setpoint = new TrapezoidProfile.State(m_elevatorSubsystem.getEncoderPosition(), 
                                                   0.0);

        addRequirements(subsystem);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() 
    {
        m_feedforward = new ElevatorFeedforward(SmartDashboard.getNumber("ks", 0.0),
                                                ElevatorConstants.kG,
                                                ElevatorConstants.kV,
                                                SmartDashboard.getNumber("ka", 0.0));

        m_pid.setPID(SmartDashboard.getNumber("kp", 0.0),
                     SmartDashboard.getNumber("ki", 0.0),
                     SmartDashboard.getNumber("kd", 0.0));

        this.setpoint = new TrapezoidProfile.State(m_elevatorSubsystem.getEncoderPosition(), 
                                                   0.0);
        this.robotIsTryingToDestroyItself = false;

        SmartDashboard.putNumber("goal", position);
    }

    @Override
    public void execute() 
    {
        // double currentHeight = m_elevatorSubsystem.getEncoderPosition();
        // double currentVelocity = m_elevatorSubsystem.getEncoderVelocity();
        // TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentHeight, currentVelocity);
        TrapezoidProfile.State nextSetpoint = m_motionProfiler.calculate(0.02, 
                                                                         setpoint, 
                                                                         new TrapezoidProfile.State(position, 0.0));
    
        double feedforwardOutput = m_feedforward.calculate(nextSetpoint.velocity);  
        double pidOutput = m_pid.calculate(m_elevatorSubsystem.getEncoderPosition(), nextSetpoint.position);

        double outputs = feedforwardOutput + pidOutput;
        this.setpoint = nextSetpoint;

        
        if(m_elevatorSubsystem.getLimitSwitch() && outputs < 0)
        {
            outputs = 0;
            robotIsTryingToDestroyItself = true;
            m_elevatorSubsystem.resetEncoder();
        }
        SmartDashboard.putNumber("elevatoroutput", outputs);
        SmartDashboard.putNumber("profiler setpoint", nextSetpoint.position);
        SmartDashboard.putNumber("elevator pos", m_elevatorSubsystem.getEncoderPosition());
        SmartDashboard.putNumber("timer", m_profilerTimer.get());
        SmartDashboard.putBoolean("bot destroy itself", robotIsTryingToDestroyItself);
        
        m_elevatorSubsystem.setMotor(outputs);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_elevatorSubsystem.setMotor(0);
    }
    

    @Override
    public boolean isFinished() 
    {
        // return m_elevatorSubsystem.getLimitSwitch() && position <= 0;
        return robotIsTryingToDestroyItself;
    }
}
