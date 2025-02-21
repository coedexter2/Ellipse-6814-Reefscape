// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.ClimbCmd;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Commands.OuttakeCmd;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.LimelightUpdate;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();
  
  private final Joystick m_Joystick = new Joystick(Constants.OIConstants.kDriverControllerPort);
  private final OuttakeSubsystem m_Out = new OuttakeSubsystem();

  private final SendableChooser<Command> autoChooser;

  private final ClimbSubsystem m_Climb = new ClimbSubsystem();

  /* 
  private final ParallelCommandGroup Swerve = new ParallelCommandGroup(new SwerveJoystickCmd(
    m_Swerve,
    () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
    () -> m_Joystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
    () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverRotAxis),
    () -> !m_Joystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx)), new LimelightUpdate(m_Swerve));
  */


  public RobotContainer() {
    m_Swerve.setDefaultCommand(new SwerveJoystickCmd(
      m_Swerve,
      () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
      () -> m_Joystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
      () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverRotAxis),
      () -> !m_Joystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx)));

    new JoystickButton(m_Joystick, 1).onTrue(new IntakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed));
    new JoystickButton(m_Joystick, 2).onTrue(new OuttakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed).withTimeout(1.5));
    
    new JoystickButton(m_Joystick, 3).onTrue(new ClimbCmd(m_Climb,
                                                                       ClimbConstants.kClimbSetpoint,
                                                                       ClimbConstants.kClimbSpeed)
                                                                       .onlyIf(() -> ((GamePhase.currentPhase == Phase.ENDGAME) || m_Joystick.getRawButton(6814))));

    new JoystickButton(m_Joystick, 4).onTrue(new AutoAlign(m_Swerve, 0));
    new JoystickButton(m_Joystick, 4).onTrue(new AutoAlign(m_Swerve, 1));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    NamedCommands.registerCommand("test", new PrintCommand("test"));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();

    /* 
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

      // 2. Generate trajectory
      //pos x = forward
      //neg x = back
      //pos y = right
      //neg y = left
      //This is all you should need for the robot container implementation of the limelight stuff: Pose2d(limelightSubsystem.getBotposeTableEntry(0), limelightSubsystem.getBotposeTableEntry(1), limelightBotposeTableEntry(5))
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, m_Swerve.getRotation2d()),
      List.of(
        new Translation2d(0.5, 0)
        ),
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        trajectoryConfig);


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_Swerve::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_Swerve::setModuleStates,
                m_Swerve);

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectory2,
                m_Swerve::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_Swerve::setModuleStates,
                m_Swerve); 

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_Swerve.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_Swerve.stopModules()));

        */
    }
}
  

