// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.Vision.AlignStates;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.util.Util;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);

  private final TunerConstants tunerConst = new TunerConstants();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Outtake outtake = new Outtake();
  Pivot pivot = new Pivot();
  Vision vision = new Vision(drivetrain);

  Superstructure superstructure = new Superstructure(elevator, intake, outtake, pivot);

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
  private PIDController pidRotationAlign = new PIDController(0.1, 0, 0);

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public RobotContainer() {
    drivetrain.configureAutoBuilder();
    configureBindings();
    initNamedCommands();
    initAutons();
  }
  
 public void initAutons() {

    // ------ path planner ------

    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    autonChooser.addOption("Leave Auton", new PathPlannerAuto("Leave"));

    autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Routine Selector", new PathPlannerAuto(getAutonomousCommand()));
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
    driver.x().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, getRotationalAlignSpeed())))
    .andThen(Commands.waitUntil(() -> Util.inRange(getRotationalAlignSpeed(), -0.1, 0.1)))
    // .andThen(Commands.print("OFHADGHAISDHFIASDHFUADS"))
    );
    driver.y().whileTrue(new InstantCommand(() -> vision.updateAlignPose()));
    // TODO: turn this into a sequence

    driver.povDown().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(-0.2, 0.0, 0.0))));
    driver.povUp().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.2, 0.0, 0.0))));
    driver.povRight().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, -0.2, 0.0))));
    driver.povLeft().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0.2, 0.0))));
    
    drivetrain.registerTelemetry(logger::telemeterize);

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driver.rightBumper().onTrue(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));

    // driver.rightTrigger().onTrue(AutoBuilder.pathfindToPose( // try a while true instead of on true
    //   vision.getPoseRight(),
    //   new PathConstraints(2, 2, 3, 2), 
    //   0.0)
    //   .alongWith(Commands.print("RIGHT"))
    //   .alongWith(new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)))
    //   .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
    // );

    // driver.leftTrigger().onTrue(AutoBuilder.pathfindToPose(
    //   vision.getPoseLeft(), 
    //   new PathConstraints(2, 2, 3, 2), 
    //   0.0)
    //   .alongWith(Commands.print("LEFT"))
    //   .alongWith(new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)))
    //   .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
    // );

    new Trigger(driver.rightTrigger()
      .whileTrue(align("right"))
      .onFalse(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    new Trigger(driver.leftTrigger()
      .whileTrue(align("left"))
      .onFalse(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    // driver.rightTrigger().whileTrue(align("right"));
    // driver.leftTrigger().whileTrue(align("left"));

    // --------------------=Operator=--------------------

    new Trigger(operator.leftTrigger())
      .whileTrue(superstructure.setState(SSStates.INTAKE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.rightTrigger())
      .whileTrue(superstructure.setState(SSStates.ALGAE_SCORE))
      .whileFalse(superstructure.setState(SSStates.STOWED)); // TODO: test this
    
    new Trigger(operator.button(8))
      .whileTrue(superstructure.setState(SSStates.EJECT))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.a())
      .whileTrue(superstructure.setState(SSStates.CORAL_1))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.b())
      .whileTrue(superstructure.setState(SSStates.CORAL_2));
      // .whileFalse(superstructure.setState(SSStates.STOWED));
    new Trigger(operator.b())
      .onFalse(new InstantCommand(()->outtake.runOuttake()).alongWith(new WaitCommand(1))
      .andThen(superstructure.setState(SSStates.STOWED)));

    new Trigger (operator.x())
      .whileTrue(superstructure.setState(SSStates.CORAL_3));
    new Trigger(operator.x())
      .onFalse(new InstantCommand(()->outtake.runOuttake()).alongWith(new WaitCommand(1))
      .andThen(superstructure.setState(SSStates.STOWED)));

    new Trigger (operator.y())
      .whileTrue(superstructure.setState(SSStates.CORAL_4));
      // .whileFalse(superstructure.setState(SSStates.STOWED));
    new Trigger(operator.x())
      .onFalse(new InstantCommand(()->outtake.runOuttake()).alongWith(new WaitCommand(1))
      .andThen(superstructure.setState(SSStates.STOWED)));

    new Trigger(operator.rightBumper())
      .whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_2))
      .whileFalse(superstructure.setState(SSStates.ALGAE_CARRY));

    new Trigger(operator.leftBumper())
      .whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_3))
      .whileFalse(superstructure.setState(SSStates.ALGAE_CARRY));

    new Trigger(operator.povUp())
      .whileTrue(new InstantCommand(() -> outtake.runOuttake()))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.povDown())
      .whileTrue(new InstantCommand(() -> outtake.runOuttake()))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    // new Trigger(operator.povLeft())
    //   .whileTrue(superstructure.setState(SSStates.ALGAE_CARRY))
    //   .whileFalse(superstructure.setState(SSStates.STOWED));
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

  
  public double getRotationalAlignSpeed() {
    vision.updatePose = true;

    double currentRotation = drivetrain.getYaw().getRadians();
    double targetRotation = currentRotation + vision.getTX();
    
    double targetSpeed = -1 * pidRotationAlign.calculate(currentRotation, targetRotation);
    // System.out.println(targetSpeed);
    return targetSpeed;
  }

  private void applyAlignSpeeds() {
    drivetrain.applyRequest(() -> new ApplyRobotSpeeds()
      .withSpeeds(new ChassisSpeeds(
          0, 
        0, 
        getRotationalAlignSpeed()
      ))
    );
  }

  // public Command align(String direction) {
  //   if (direction.equals("left")) {
  //     return Commands.sequence(
  //       new InstantCommand(() -> vision.updateAlignPose()), 
  //       new InstantCommand(() -> applyAlignSpeeds()),
  //       new WaitUntilCommand(() -> Util.inRange(vision.getTX(), 0.1, 0.1)),
  //       new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)),

  //       new RunCommand(() -> 
  //         AutoBuilder.pathfindToPose(
  //           vision.getPoseRight(),
  //           new PathConstraints(2, 2, 3, 2), 
  //           0.0
  //         )
  //       .andThen(Commands.print("LEFT"))
  //       .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
  //       )
  //     );
  //   } else if (direction.equals("right")) {
  //     return Commands.sequence(
  //       new InstantCommand(() -> vision.updateAlignPose()), 
  //       new InstantCommand(() -> applyAlignSpeeds()),
  //       new WaitUntilCommand(() -> Util.inRange(vision.getTX(), 0.1, 0.1)),
  //       new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)),

  //       new RunCommand(() -> AutoBuilder.pathfindToPose(
  //         vision.getPoseLeft(),
  //         new PathConstraints(2, 2, 3, 2), 
  //         0.0)
  //         .andThen(Commands.print("RIGHT"))
  //         .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
  //       )
  //     );
  //   } else {
  //     return new InstantCommand(() -> System.out.println("ALIGN FAILED"));
  //   }
  // }

  public Command align(String direction) {
    if (direction.equals("left")) {
      return Commands.either(
        new RunCommand(() -> applyAlignSpeeds()),
        Commands.sequence(
          new InstantCommand(() -> vision.updateAlignPose()), 
          AutoBuilder.pathfindToPose(
            vision.getPoseRight(),
            new PathConstraints(2, 2, 3, 2), 
    0.0
          )
            .alongWith(Commands.print("RIGHT"))
            .alongWith(new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)))
            .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
        ),
        () -> !Util.inRange(vision.getTX(), 0.1, 0.1)
      );
    } else if (direction.equals("right")) {
      return Commands.either(
        new RunCommand(() -> applyAlignSpeeds()),
        Commands.sequence(
          new InstantCommand(() -> vision.updateAlignPose()), 
          AutoBuilder.pathfindToPose(
            vision.getPoseLeft(),
            new PathConstraints(2, 2, 3, 2), 
    0.0
          )
            .alongWith(Commands.print("RIGHT"))
            .alongWith(new InstantCommand(() -> vision.setAlignState(AlignStates.ALIGNING)))
            .andThen(new InstantCommand(() -> vision.setAlignState(AlignStates.NONE)))
        ),
        () -> !Util.inRange(vision.getTX(), 0.1, 0.1)
      );
    } else {
      return new InstantCommand(() -> System.out.println("ALIGN FAILED"));
    }
  }
}

  // private Command pathfind(String direction) {
  //   if (direction.equals("right")) {
  //     return Commands.sequence(
  //       new FunctionalCommand(
  //         () -> applyAlignSpeeds(),
  //         () -> applyAlignSpeeds(),
  //         () -> System.out.println("pathfinding"),
  //         Util.inRange(vision.getTX(), 0.1, 0.1),
  //         drivetrain.getState())
  //     );
  //   }

