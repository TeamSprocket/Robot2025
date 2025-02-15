// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);

  private final TunerConstants tunerConst = new TunerConstants();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  Superstructure superstructure = new Superstructure();
  Vision limelight = new Vision(drivetrain);

  // ------- Swerve Generated -------
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    drivetrain.configureAutoBuilder();
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

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.6) // Drive counterclockwise with negative X (left)
        )
    );

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    ));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // driver.y().onTrue(AutoBuilder.pathfindToPose(
    //   limelight.getPoseRight(),
    //     new PathConstraints(2, 2, 3, 2), 
    //     0.0)
    //     .andThen(Commands.print("RIGHT")));

    // driver.x().onTrue(AutoBuilder.pathfindToPose(
    //   limelight.getPoseLeft(), 
    //   new PathConstraints(2, 2, 3, 2), 
    //   0.0)
    //   .andThen(Commands.print("LEFT")));
      driver.rightBumper().whileTrue(drivetrain.autopath());
      driver.x().onTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d(1.177, 2.197, Rotation2d.fromDegrees(0)))));
    drivetrain.registerTelemetry(logger::telemeterize);

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
}