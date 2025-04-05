// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.ClimbHomeCmd;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_ClimbHomeCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(DriverStation.isAutonomous())
    {
      GamePhase.currentPhase = Phase.AUTONOMOUS;
    }
    else
    {
      if(DriverStation.getMatchTime() > 69)
      {
        GamePhase.currentPhase = Phase.ENDGAME;
      } 
      else
      {
        GamePhase.currentPhase = Phase.AUTONOMOUS;
      }
    }

    

    SmartDashboard.putData("Scheduler",CommandScheduler.getInstance());
    SmartDashboard.putNumber("matchtime", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Beambreaker", m_robotContainer.m_Out.isBroken());
    SmartDashboard.putNumber("Arm Rotation", m_robotContainer.m_Arm.getEncoderPosition());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // m_ClimbHomeCommand = m_robotContainer.getClimbHomeCommand();
    // m_ClimbHomeCommand.schedule();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
