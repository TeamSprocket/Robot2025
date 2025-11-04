// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
  // private final CommandXboxController driver = new CommandXboxController(0); // My joystick

  /*
   * 7. From here, define your subsystem as an object. If an error pops up, use the lightbulb in the corner
   * to import the subsystem. Also, uncomment the above piece of code
   */


  private Timer timer = new Timer();


  public RobotContainer() {
    configureBindings();
  }
  

  public void configureBindings() {
    /*
     * 8. Here, use the driver controller to bind the a button to powering the motor. Once you've done that
     * and gotten check, speak to a lead to see if you can test the function. Congratulations, you've completed the module!
     */

    // --------------------=Driver=--------------------
  }


}


