// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.OutTakeCmd;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();
  private final OuttakeSubsystem m_Out = new OuttakeSubsystem();
  private final Joystick m_Joystick = new Joystick(Constants.OIConstants.kDriverControllerPort);
  

  public RobotContainer() {
    m_Swerve.setDefaultCommand(new SwerveJoystickCmd(
      m_Swerve,
      () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverXAxis),
      () -> m_Joystick.getRawAxis(Constants.OIConstants.kDriverYAxis),
      () -> -m_Joystick.getRawAxis(Constants.OIConstants.kDriverRotAxis),
      () -> !m_Joystick.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx)));
      new JoystickButton(m_Joystick, 0).whileTrue(new OutTakeCmd(m_Out,Constants.OuttakeConstants.kOuttakeSpeed));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
