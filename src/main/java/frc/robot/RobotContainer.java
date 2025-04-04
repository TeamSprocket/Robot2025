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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SSStates;
import frc.robot.subsystems.Vision.AlignStates;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import static edu.wpi.first.units.Units.*;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

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
  Climb climb = new Climb();

  public final AutoFactory autoFactory;

  double speedMultiplier = 1.0;
  Superstructure superstructure = new Superstructure(elevator, intake, outtake, pivot, climb);

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

  private Timer timer = new Timer();
  private Pose2d targetPose = new Pose2d();

  private AutoChooser autoChooser;

  public SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  double alignTimeout = 2.0; //TUNE
  double intakeTimeout = 1.5; //TUNE
  double scoreTimeout = 0.75; //TUNE

  public RobotContainer() {
    // drivetrain.configureAutoBuilder();
    autoFactory = new AutoFactory(
      drivetrain::getAutoBuilderPose,
      drivetrain::resetTeleopPose,
      drivetrain::followTrajectory,
      true,
      drivetrain
    );
    configureBindings();
    
    // initNamedCommands();
    initAutons();
  }
  
 public void initAutons() {
    

    autoChooser = new AutoChooser();

    autoChooser.addRoutine("routine", this::testAuton);

    // autoChooser.addCmd("toReef", this::goToReef);
    // autoChooser.addCmd("toReefL4", this::moveToReefL4);
    // autoChooser.addCmd("testCircle", this::testCircle);
    // autoChooser.addCmd("rotationTest", this::rotationTest);
    // autoChooser.addCmd("test1", this::test);
    // autoChooser.addCmd("test2", this::test2);
    // autoChooser.addCmd("testBoth", this::testBoth);

    SmartDashboard.putData("Select Auto", autoChooser);
    
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());



  }

  // public AutoRoutine routine() {
  //   AutoRoutine routine = autoFactory.newRoutine("Routine");
  //   AutoTrajectory traj1 = routine.trajectory("test path");
  //   AutoTrajectory traj2 = routine.trajectory("back out");

  //   routine.active().onTrue(
  //     Commands.sequence(
  //       new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
  //       traj1.resetOdometry(),
  //       traj1.cmd()
  //     )
  //   );

  public AutoRoutine testAuton() {
    AutoRoutine routine = autoFactory.newRoutine("preloadScoreBL_L4"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STM_BL"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BLL_SL");
    AutoTrajectory traj3 = routine.trajectory("SL_BL");

    routine.active().onTrue(
      Commands.sequence(
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );

    
    // traj1.active().whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_3));
    traj1.done().onTrue(scoreL4Left().andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());
    return routine;
  }


  public Command alignLeft() {
    return choreoAlignLeft().withTimeout(alignTimeout).andThen(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));
  }

  public Command alignRight() {
    return choreoAlignRight().withTimeout(alignTimeout).andThen(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));
  }

  public Command scoreL2Left() {
    return alignLeft().andThen(superstructure.setState(SSStates.CORAL_2)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command scoreL2Right() {
    return alignRight().andThen(superstructure.setState(SSStates.CORAL_2)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command scoreL3Left() {
    return alignLeft().andThen(superstructure.setState(SSStates.CORAL_3)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command scoreL3Right() {
    return alignRight().andThen(superstructure.setState(SSStates.CORAL_3)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command scoreL4Left() {
    return alignLeft().andThen(superstructure.setState(SSStates.CORAL_4)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command scoreL4Right() {
    return alignRight().andThen(superstructure.setState(SSStates.CORAL_4)).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command intake() {
    return superstructure.setState(SSStates.INTAKE).andThen(Commands.waitSeconds(intakeTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  public void initNamedCommands() {
  }

  public Command choreoAlignLeft() {
    return drivetrain.applyRequest(
        () -> new ApplyFieldSpeeds()
          .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsLeft()[0], vision.getAlignOffsetsLeft()[1], vision.getRotationalAlignSpeedLeft()))
        ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)));
  }
  public Command choreoAlignRight() {
    return drivetrain.applyRequest(
      () -> new ApplyFieldSpeeds()
        .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsRight()[0], vision.getAlignOffsetsRight()[1], vision.getRotationalAlignSpeedRight()))
    ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)));
  }

  public void configureBindings() {
    // --------------------=Driver=--------------------

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * speedMultiplier * 0.5) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * speedMultiplier * 0.5) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.6
            ) // Drive counterclockwise with negative X (left)
        )
    );

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    ));

    driver.povDown().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(-0.5, 0.0, 0.0))));
    driver.povUp().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))));
    driver.povRight().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, -0.5, 0.0))));
    driver.povLeft().whileTrue(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0.5, 0.0))));
    
    drivetrain.registerTelemetry(logger::telemeterize);

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driver.rightBumper().onTrue(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));

    driver.y().onTrue(new InstantCommand(() -> vision.updateAlignPose()));

    // driver.y().onTrue(new InstantCommand(() -> speedMultiplier = 0.3))
    //   .onFalse(new InstantCommand(() -> speedMultiplier = 0.1));

    driver.rightTrigger().whileTrue(
      drivetrain.applyRequest(
        () -> new ApplyFieldSpeeds()
          .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsRight()[0], vision.getAlignOffsetsRight()[1], vision.getRotationalAlignSpeedRight()))
      ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)))
    );

    driver.rightTrigger().onFalse(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    driver.leftTrigger().whileTrue(
      drivetrain.applyRequest(
        () -> new ApplyFieldSpeeds()
          .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsLeft()[0], vision.getAlignOffsetsLeft()[1], vision.getRotationalAlignSpeedLeft()))
        ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)))
    );

    driver.leftTrigger().onFalse(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    // --------------------=Operator=--------------------

    new Trigger(operator.leftTrigger())
      .whileTrue(superstructure.setState(SSStates.INTAKE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.a())
      .whileTrue(superstructure.setState(SSStates.CORAL_1))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.b())
      .whileTrue(superstructure.setState(SSStates.CORAL_2))
      .onFalse(superstructure.setState(SSStates.STOWED));
    
    new Trigger (operator.x())
      .whileTrue(superstructure.setState(SSStates.CORAL_3))
      .onFalse(superstructure.setState(SSStates.STOWED));
    
    new Trigger(() -> climb.notAtPosition())
      .whileTrue(superstructure.setState(SSStates.STOWED));

    new Trigger (operator.y())
      .whileTrue(superstructure.setState(SSStates.CORAL_4))
      .onFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.rightBumper())
      .whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_2))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.leftBumper())
      .whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_3))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.povUp())
      .whileTrue(new InstantCommand(() -> outtake.runOuttake()))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.povDown())
      .whileTrue(superstructure.setState(SSStates.EJECT))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger (operator.button(8).and(() -> climb.notAtPosition()))
      .whileTrue(superstructure.setState(SSStates.CLIMB))
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

  public void waitTime(double duration) {
    Timer timer = new Timer();
    // timer.delay(duration);
    timer.start();
    while (timer.get() < duration) {

    }
    timer.stop();
    timer.reset();
    }
  }
