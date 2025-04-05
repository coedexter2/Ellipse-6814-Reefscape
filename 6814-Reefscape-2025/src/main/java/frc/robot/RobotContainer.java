// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Newton;

import java.util.List;

import javax.swing.GroupLayout.Alignment;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Commands.ArmCmd;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.ClimbCmd;
import frc.robot.Commands.ClimbHomeCmd;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.GroundIntakeCmd;
import frc.robot.Commands.IntakeClearenceCmd;
import frc.robot.Commands.IntakeClearenceCmd;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Commands.OuttakeCmd;
import frc.robot.Commands.ReefLockCmd;
import frc.robot.Commands.SecureCoral;
import frc.robot.Commands.SourceLockCmd;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.LimelightUpdate;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.GroundIntakeSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();
  public final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
  private final Joystick m_DriveJoystick = new Joystick(Constants.OIConstants.kDriverControllerPort);
  public final Joystick m_ElevatorJoystick = new Joystick(OIConstants.kElevatorJoystickPort);
  public final OuttakeSubsystem m_Out = new OuttakeSubsystem();
  private final ArmSubsystem m_Arm = new ArmSubsystem();
private final GroundIntakeSubsystem m_Ground = new GroundIntakeSubsystem();

  private final SendableChooser<Command> autoChooser;

  public final Command m_limelightUpdater = new LimelightUpdate(m_Swerve);

  // private final ClimbSubsystem m_Climb = new ClimbSubsystem();

  
  private final ParallelCommandGroup Swerve = new ParallelCommandGroup(new SwerveJoystickCmd(
    m_Swerve, m_Elevator,
    () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
    () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
    () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverRotAxis),
    () -> m_DriveJoystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx),
    () -> m_DriveJoystick.getRawAxis(3)), new LimelightUpdate(m_Swerve));

    private final ParallelCommandGroup LockedSwerve = new ParallelCommandGroup(new SourceLockCmd(
      m_Swerve, m_Elevator,
      () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
      () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverYAxis), 
      () -> m_DriveJoystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx),
      () -> m_DriveJoystick.getRawAxis(3)));

      private final ParallelCommandGroup ReefLockSwerve = new ParallelCommandGroup(new ReefLockCmd(
        m_Swerve, m_Elevator,
        () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
        () -> -m_DriveJoystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
        () -> m_DriveJoystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx),
        () -> m_DriveJoystick.getRawAxis(3)), new LimelightUpdate(m_Swerve));

   
  // public Command ElevateOutOne = new ElevatorCommand(m_Elevator, Constants.ElevatorConstants.kFirstLevel)
  // .andThen(new WaitCommand(0.5).andThen(new OuttakeCmd(m_Out, Constants.OuttakeConstants.kOuttakeSpeed)));
  
  // public Command ElevateOutTwo = new ElevatorCommand(m_Elevator, Constants.ElevatorConstants.kSecondLevel)
  // .andThen(new WaitCommand(0.5).andThen(new OuttakeCmd(m_Out, Constants.OuttakeConstants.kOuttakeSpeed)));

  // public Command ElevateOutThird = new ElevatorCommand(m_Elevator, Constants.ElevatorConstants.kThirdLevel)
  // .andThen(new WaitCommand(0.5).andThen(new OuttakeCmd(m_Out, Constants.OuttakeConstants.kOuttakeSpeed)));

  // public Command ElevateOutFourth = new ElevatorCommand(m_Elevator, Constants.ElevatorConstants.kFourthLevel)
  // .andThen(new WaitCommand(0.5).andThen(new OuttakeCmd(m_Out, Constants.OuttakeConstants.kOuttakeSpeed)));
  

  public RobotContainer() {

    NamedCommands.registerCommand("Outtake", new OuttakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed).withTimeout(0.65));
    NamedCommands.registerCommand("Intake", new IntakeCmd(m_Out, OuttakeConstants.kIntakeSpeed, 2));

    NamedCommands.registerCommand("Elevator 1", new ElevatorCommand(m_Elevator, ElevatorConstants.kFirstLevel));
    NamedCommands.registerCommand("Elevator 2", new ElevatorCommand(m_Elevator, ElevatorConstants.kSecondLevel));
    NamedCommands.registerCommand("Elevator 3", new ElevatorCommand(m_Elevator, ElevatorConstants.kThirdLevel));
    NamedCommands.registerCommand("Elevator 4", new ElevatorCommand(m_Elevator, ElevatorConstants.kFourthLevel));
    NamedCommands.registerCommand("Elevator Src", new ElevatorCommand(m_Elevator, ElevatorConstants.kSourceIntake));

    NamedCommands.registerCommand("Align Left", new AutoAlign(m_Swerve, ReefAlignment.LEFT).withTimeout(3));
    NamedCommands.registerCommand("Align Right", new AutoAlign(m_Swerve, ReefAlignment.RIGHT));

  NamedCommands.registerCommand("Limelight", new LimelightUpdate(m_Swerve));


   m_Swerve.setDefaultCommand(Swerve);


   new JoystickButton(m_DriveJoystick, 1).whileTrue(LockedSwerve);
   new JoystickButton(m_DriveJoystick, 3).whileTrue(ReefLockSwerve);

    new JoystickButton(m_ElevatorJoystick, 4).onTrue(new IntakeCmd(m_Out, OuttakeConstants.kIntakeSpeed, 1).andThen(new ElevatorCommand(m_Elevator, Units.inchesToMeters(5))).andThen(null));
    new JoystickButton(m_ElevatorJoystick, 2).onTrue(new IntakeCmd(m_Out, OuttakeConstants.kIntakeSpeed, 2).alongWith(new ElevatorCommand(m_Elevator, ElevatorConstants.kSourceIntake)));
    new JoystickButton(m_ElevatorJoystick, 3).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kSourceIntake));
    //new JoystickButton(m_ElevatorJoystick, 2).onTrue(new IntakeCmd(m_Out, OuttakeConstants.kIntakeSpeed).andThen(new IntakeClearenceCmd(m_Out, OuttakeConstants.kIntakeSpeed, OuttakeConstants.kScorePos)));

    new JoystickButton(m_ElevatorJoystick, 1).onTrue(new OuttakeCmd(m_Out, OuttakeConstants.kOuttakeSpeed).withTimeout(0.5));

    // new JoystickButton(m_ElevatorJoystick, 2).onTrue(new IntakeCmd(m_Out, OuttakeConstants.kIntakeSpeed).alongWith(new ElevatorCommand(m_Elevator, ElevatorConstants.kSourceIntake)));
    // // new JoystickButton(m_ElevatorJoystick, 1).onTrue(new OuttakeCmd(m_Out, 0.5).withTimeout(0.4).andThen(new WaitCommand(0.2).andThen(new OuttakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed).withTimeout(1))));
    // new JoystickButton(m_ElevatorJoystick, 1).onTrue(new OuttakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed).withTimeout(1));

    // new POVButton(m_ElevatorJoystick, 0).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kFourthLevel).
    //                                           alongWith(new IntakeClearence(m_Out,m_Elevator,OuttakeConstants.kIntakeSpeed))
    //                                           .andThen(new WaitCommand(1)).andThen(new IntakeCmd(m_Out, -OuttakeConstants.kIntakeSpeed)));

    // new POVButton(m_ElevatorJoystick, 90).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kThirdLevel).
    //                                           alongWith(new IntakeClearence(m_Out,m_Elevator,OuttakeConstants.kIntakeSpeed))
    //                                           .andThen(new WaitCommand(1)).andThen(new IntakeCmd(m_Out, -OuttakeConstants.kIntakeSpeed)));

    // new POVButton(m_ElevatorJoystick, 270).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kSecondLevel).
    //                                           alongWith(new IntakeClearence(m_Out,m_Elevator,OuttakeConstants.kIntakeSpeed))
    //                                           .andThen(new WaitCommand(1)).andThen(new IntakeCmd(m_Out, -OuttakeConstants.kIntakeSpeed)));

    new POVButton(m_ElevatorJoystick, 0).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kFourthLevel));

    new POVButton(m_ElevatorJoystick, 90).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kThirdLevel));

    new POVButton(m_ElevatorJoystick, 270).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kSecondLevel));

    new POVButton(m_ElevatorJoystick, 180).onTrue(new ElevatorCommand(m_Elevator, ElevatorConstants.kFirstLevel));

    // new JoystickButton(m_ElevatorJoystick, 2).onTrue(new ClimbCmd(m_Climb, ClimbConstants.kClimbSetpoint, ClimbConstants.kClimbSpeed).onlyIf(()->m_ElevatorJoystick.getRawAxis(3) > 0.5));


    new JoystickButton(m_DriveJoystick, 5).whileTrue(new AutoAlign(m_Swerve, ReefAlignment.LEFT));
    new JoystickButton(m_DriveJoystick, 6).whileTrue(new AutoAlign(m_Swerve, ReefAlignment.RIGHT));
    
    // new JoystickButton(m_ElevatorJoystick, 7).onTrue(new ArmCmd(m_Arm, ArmConstants.kUpPose).andThen(new IntakeCmd(m_Out, 0)).withTimeout(1));
    // new JoystickButton(m_ElevatorJoystick, 5).onTrue(new SecureCoral(m_Ground, GroundIntakeConstants.kGroundIntakeSpeed, GroundIntakeConstants.kL1Pose)
    //                                                     .andThen(new ArmCmd(m_Arm, ArmConstants.kL1Pose))); 
    // new JoystickButton(m_ElevatorJoystick, 8).onTrue(new ArmCmd(m_Arm, ArmConstants.kDownPose)); 
    
    // new JoystickButton(m_ElevatorJoystick, 6).onTrue(new ArmCmd(m_Arm, ArmConstants.kIntakePose).
    //                         alongWith(new GroundIntakeCmd(m_Ground, GroundIntakeConstants.kGroundIntakeSpeed)));
    // new JoystickButton(m_ElevatorJoystick, 3).onTrue(new GroundIntakeCmd(m_Ground, GroundIntakeConstants.kGroundOuttakeSpeed).
    //                                                               withTimeout(GroundIntakeConstants.kOuttakeTimeout));
    // new JoystickButton(m_DriveJoystick, 2).whileTrue(new OuttakeCmd(m_Out, 1));

    // SmartDashboard.putNumber("ks", 0);
    // SmartDashboard.putNumber("kg", 0);
    // SmartDashboard.putNumber("kv", 0);
    // SmartDashboard.putNumber("ka", 0);
    // SmartDashboard.putNumber("kp", 0);
    // SmartDashboard.putNumber("ki", 0);
    // SmartDashboard.putNumber("kd", 0);

    // SmartDashboard.putNumber("l2", 0);
    HttpCamera httpCamera = new HttpCamera("Limelight Camera", "http://limelight.local:5800");
    HttpCamera intakeCamera = new HttpCamera("Intake Camera", "http://limelight-back.local:5800");
    
    SmartDashboard.putNumber("ArmKs", 0);
    SmartDashboard.putNumber("ArmKg", 0);
    SmartDashboard.putNumber("ArmKv", 0);
    SmartDashboard.putNumber("ArmKa", 0);

    SmartDashboard.putNumber("ArmKp", 0);
    SmartDashboard.putNumber("ArmKi", 0);
    SmartDashboard.putNumber("ArmKd", 0);

    CameraServer.addCamera(httpCamera);
    CameraServer.addCamera(intakeCamera);
   
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getClimbHomeCommand()
  {
    // return new ClimbHomeCmd(m_Climb, ClimbConstants.kClimbHomeSpeed);
    return new PrintCommand("AAAAAAAAAAAA");
  }

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
  

