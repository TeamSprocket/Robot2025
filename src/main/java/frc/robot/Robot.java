// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure.SSStates;

public class Robot extends TimedRobot {
  private final CommandXboxController controller = new CommandXboxController(0);
  final TalonFX climbTest = new TalonFX(10);
  final TalonFX climbTest2 = new TalonFX(11);
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  // private void configMotors(){
  //   TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  //   climbTest.getConfigurator().apply(talonFXConfigs, 0);
  //   climbTest2.getConfigurator().apply(talonFXConfigs, 0);
  // }


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

    
    m_robotContainer.getSuperstructure().setState(SSStates.testElevator);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  
  // controller.a().whileTrue(new InstantCommand(() -> climbTest.set(-0.1)));
  // controller.a().whileFalse(new InstantCommand(() -> climbTest.set(0.0)));
  // controller.b().whileTrue(new InstantCommand(() -> climbTest2.set(0.1)));
  // controller.b().whileFalse(new InstantCommand(() -> climbTest2.set(0.0)));

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