// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);

  Superstructure superstructure = new Superstructure();

  // ------- Swerve Generated -------

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    initNamedCommands();
    initAutons();
  }
  
 public void initAutons() {

    // ------ path planner ------

    SmartDashboard.putData("Auto Routine Selector", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void initNamedCommands() {

  }

  public void configureBindings() {
    // --------------------=Driver=--------------------

    // --------------------=Operator=--------------------

    new Trigger(operator.x())
      .whileTrue(superstructure.setState(SSStates.NONE))
      .whileFalse(superstructure.setState(SSStates.STOWED));
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public Command rumbleControllers() {
    return Commands.runOnce(() ->
      CommandScheduler.getInstance().schedule(
        Commands.sequence(
          Commands.waitSeconds(0.5),
          Commands.runOnce(() -> {
              operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
              driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
          }),
          Commands.waitSeconds(0.5),
          Commands.runOnce(() -> {
              operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
              driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
          })
        )
      )
    );
  }

  // public Command alignSwerveCommand() {
  //   return drivetrain.applyRequest(() -> headingLock.withSpeeds(vision.getHeadingLockSpeed()));
  // }
}