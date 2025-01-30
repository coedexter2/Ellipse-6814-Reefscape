// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();

  private final Joystick m_Joysitck = new Joystick(Constants.OIConstants.kDriverControllerPort);

  

  public RobotContainer() {
    m_Swerve.setDefaultCommand(new SwerveJoystickCmd(
      m_Swerve,
      () -> -m_Joysitck.getRawAxis(Constants.OIConstants.kDriverXAxis),
      () -> m_Joysitck.getRawAxis(Constants.OIConstants.kDriverYAxis),
      () -> -m_Joysitck.getRawAxis(Constants.OIConstants.kDriverRotAxis),
      () -> !m_Joysitck.getRawButton(Constants.OIConstants.kDriverFieldOrientedButtonIdx)));


    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
