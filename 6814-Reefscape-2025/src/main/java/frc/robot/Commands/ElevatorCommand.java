package frc.robot.Commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    // ===== Feedforward stuff =====
    private final Timer m_profilerTimer = new Timer();
    private final TrapezoidProfile m_motionProfiler = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, 
                                                                                                            ElevatorConstants.kMaxAcceleration));
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, 
                                                                              ElevatorConstants.kG, 
                                                                              ElevatorConstants.kV,
                                                                              ElevatorConstants.kA);
    private TrapezoidProfile.State initialState;
    
    // ===== PID stuff =====
    private final PIDController m_pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double position;

    public ElevatorCommand(ElevatorSubsystem subsystem, double position)
    {
        this.m_elevatorSubsystem = subsystem;
        this.position = position;
        this.initialState = new TrapezoidProfile.State(m_elevatorSubsystem.getEncoderPosition(), 
                                                       m_elevatorSubsystem.getEncoderVelocity());

        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() 
    {
        m_profilerTimer.reset();
        m_profilerTimer.start();
    }

    @Override
    public void execute() 
    {
        TrapezoidProfile.State profiledSetpoint = m_motionProfiler.calculate(m_profilerTimer.get(), 
                                                                             initialState, 
                                                                             new TrapezoidProfile.State(position, 0.0));

        double feedforwardOutput = m_feedforward.calculate(profiledSetpoint.velocity);
        double pidOutput = m_pid.calculate(m_elevatorSubsystem.getEncoderPosition(), profiledSetpoint.position);
        
        m_elevatorSubsystem.setMotor(feedforwardOutput + pidOutput);
    }

    @Override
    public void end(boolean interrupted)
    {
        
    }
    

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
