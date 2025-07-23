// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private boolean run = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Timer.delay(0.5);

    // logging
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Timer.delay(0.05);
    // m_robotContainer.getSuperstructure().setState(SSStates.STOWED);

    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
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
    Timer.delay(0.05);

    // Timer.delay(0.15);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.getSuperstructure().setState(SSStates.STOWED);

    
  }

  @Override
  public void teleopPeriodic() {
 
  }

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
