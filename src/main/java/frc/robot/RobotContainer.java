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

  double alignTimeout = 1.25; //TUNE ALSO LOWER
  double intakeTimeout = 3.0; //TUNE
  double scoreTimeout = 0.20; //TUNE ALSO LOWER

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
    initAutons();
  }
  
 public void initAutons() {
    

    autoChooser = new AutoChooser();

    //---------universal-------------
 
    autoChooser.addRoutine("STL_LEAVE", this::STL_LEAVE);
    autoChooser.addRoutine("STM_LEAVE", this::STM_LEAVE);
    autoChooser.addRoutine("STR_LEAVE", this::STR_LEAVE);

    //---------champs---------
    autoChooser.addRoutine("right2CoralAuton", this::right2CoralAuton);
    autoChooser.addRoutine("CENTER_LEFT", this::CENTER_LEFT);
    autoChooser.addRoutine("left2CoralAuton", this::left2CoralAuton);
    // autoChooser.addRoutine("notmatch4_Champs", this::match4_Champs);

    //---------cvr------------

    // autoChooser.addRoutine("match31", this::STB_BR_L4L);
    // autoChooser.addRoutine("match20", this::match20);
    // autoChooser.addRoutine("match37", this::match37);
    // autoChooser.addRoutine("match48", this::match48);
    // autoChooser.addRoutine("match 63", this::match63);
    // autoChooser.addRoutine("SL_TEST", this::SL_TEST);

    //---------test----------

    // autoChooser.addRoutine("testPID", this::testPID);
    // autoChooser.addRoutine("2mf", this::twometerforward);
    // autoChooser.addRoutine("2mft", this::twometerforwardturn);
    SmartDashboard.putData("Select Auto", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  //MOVE EVERYTHING TO ANOTHER FILE SO THAT ROBOT CONTAINER IS NOT CLUTERED

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

  // public AutoRoutine STM_BL_L4L_SL_BL_L4R() {
  //   AutoRoutine routine = autoFactory.newRoutine("STM_BL_L4L_SL_BL_L4R"); //ROUTINE NAME
  //   AutoTrajectory traj1 = routine.trajectory("STM_BL"); //LOAD ALL PATHS HERE
  //   AutoTrajectory traj2 = routine.trajectory("BLL_SL");
  //   AutoTrajectory traj3 = routine.trajectory("SL_BL");

  //   routine.active().onTrue(
  //     Commands.sequence(
  //       new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
  //       superstructure.setState(SSStates.STOWED),
  //       traj1.resetOdometry(),
  //       traj1.cmd()
  //     )
  //   );
  //   traj1.done().onTrue(scoreL4Left().andThen(traj2.cmd()));
  //   traj2.done().onTrue(intake().andThen(traj3.cmd()));
  //   traj3.done().onTrue(scoreL4Right());
  //   return routine;
  // }

  // public AutoRoutine STR_BR_L4L_SR_BR_L4R() {
  //   AutoRoutine routine = autoFactory.newRoutine("STR_BR_L4L_SR_BR_L4R"); //ROUTINE NAME
  //   AutoTrajectory traj1 = routine.trajectory("STR_BR"); //LOAD ALL PATHS HERE
  //   AutoTrajectory traj2 = routine.trajectory("BRL_SR");
  //   AutoTrajectory traj3 = routine.trajectory("SR_BR");

  //   routine.active().onTrue(
      
  //     Commands.sequence(
  //       new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
  //       superstructure.setState(SSStates.STOWED),
  //       traj1.resetOdometry(),
  //       traj1.cmd()
  //     )
  //   );

    
  //   // traj1.active().whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_3));
  //   traj1.done().onTrue(scoreL4Left().andThen(traj2.cmd()));
  //   traj2.done().onTrue(intake().andThen(traj3.cmd()));
  //   traj3.done().onTrue(scoreL4Right());
  //   return routine;
  // }

  // public AutoRoutine STR_BM_L4L_SR_BM_L4R() {
  //   AutoRoutine routine = autoFactory.newRoutine("STR_BM_L4L_SR_BM_L4R"); //ROUTINE NAME
  //   AutoTrajectory traj1 = routine.trajectory("STR_BM"); //LOAD ALL PATHS HERE
  //   AutoTrajectory traj2 = routine.trajectory("BML_SR");
  //   AutoTrajectory traj3 = routine.trajectory("SR_BM");

  //   routine.active().onTrue(
  //     Commands.sequence(
  //       new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
  //       superstructure.setState(SSStates.STOWED),
  //       traj1.resetOdometry(),
  //       traj1.cmd()
  //     )
  //   );

    
  //   traj1.active().whileTrue(superstructure.setState(SSStates.ALGAE_REMOVE_3));
  //   traj1.done().onTrue(scoreL4Left().andThen(traj2.cmd()));
  //   traj2.done().onTrue(intake().andThen(traj3.cmd()));
  //   traj3.done().onTrue(scoreL4Right());
  //   return routine;
  // }

  public AutoRoutine STL_LEAVE() {
    AutoRoutine routine = autoFactory.newRoutine("STL_LEAVE"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STL_LEAVE"); //LOAD ALL PATHS HERE


    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine STM_LEAVE() {
    AutoRoutine routine = autoFactory.newRoutine("STM_LEAVE"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STM_LEAVE"); //LOAD ALL PATHS HERE


    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine STR_LEAVE() {
    AutoRoutine routine = autoFactory.newRoutine("STR_LEAVE"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STR_LEAVE"); //LOAD ALL PATHS HERE


    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine match4_Champs() { // TODO: have eric check lmao
    AutoRoutine routine = autoFactory.newRoutine("match4_Champs"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STM_BL"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BLR_SL");
    AutoTrajectory traj3 = routine.trajectory("SL_FL");

    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Left().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.resetOdometry()).andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());
  
    return routine;
  }

  public AutoRoutine right2CoralAuton() { // TODO: have eric check lmao
    AutoRoutine routine = autoFactory.newRoutine("right2CoralAuton"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STB_BR"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BRR_SR");
    AutoTrajectory traj3 = routine.trajectory("SR_FR");

    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Left().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.resetOdometry()).andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());
  
    return routine;
  }

  public AutoRoutine left2CoralAuton() { // TODO: have eric check lmao
    AutoRoutine routine = autoFactory.newRoutine("left2CoralAuton"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STM_BL"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BLR_SL");
    AutoTrajectory traj3 = routine.trajectory("SL_FL");

    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Left().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.resetOdometry()).andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());
  
    return routine;
  }

  public AutoRoutine testPID() {
    AutoRoutine routine = autoFactory.newRoutine("testPID"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("testPID"); //LOAD ALL PATHS HERE


    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );
    return routine;
  }

  public AutoRoutine STB_BR_L4L() {
    AutoRoutine routine = autoFactory.newRoutine("STB_BM_L4L"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STB_BR"); //LOAD ALL PATHS HERE
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Right());
    return routine;
  }

  public AutoRoutine match20() {
    AutoRoutine routine = autoFactory.newRoutine("match20"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("ST_FIELD_MIDDLE_LEAVE"); //LOAD ALL PATHS HERE
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),
        // Commands.waitSeconds(0.5),

        traj1.cmd()
      )
    );

    // traj1.done().onTrue(scoreL4Right());
    return routine;
  }

  public AutoRoutine match37() {
    AutoRoutine routine = autoFactory.newRoutine("match20"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("ST_FIELD_MIDDLE_LEAVE"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BML_SL");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Right().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    // traj2.done().onTrue(intake());

    return routine;
  }

  public AutoRoutine SL_TEST() {
    AutoRoutine routine = autoFactory.newRoutine("match20"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("ST_FIELD_MIDDLE_LEAVE"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BML_SL");
    AutoTrajectory traj3 = routine.trajectory("SL_FM");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Right().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());

    return routine;
  }

  public AutoRoutine match48() {
    AutoRoutine routine = autoFactory.newRoutine("match48"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("ST_FIELD_MIDDLE_LEAVE"); //LOAD ALL PATHS HERE
    // AutoTrajectory traj2 = routine.trajectory("BML_SL");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Right());
    // traj2.done().onTrue(intake());

    return routine;
  }

  public AutoRoutine CENTER_LEFT() {
    AutoRoutine routine = autoFactory.newRoutine("CENTER_LEFT"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("ST_FIELD_MIDDLE_LEAVE"); //LOAD ALL PATHS HERE
    // AutoTrajectory traj2 = routine.trajectory("BLL_SL");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Left());
    // traj2.done().onTrue(intake());

    return routine;
  }

  public AutoRoutine twometerforward() {
    AutoRoutine routine = autoFactory.newRoutine("2mf"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("2mforward"); //LOAD ALL PATHS HERE
    // AutoTrajectory traj2 = routine.trajectory("BLL_SL");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    // traj1.done().onTrue(scoreL4Left());
    // traj2.done().onTrue(intake());

    return routine;
  }

  public AutoRoutine twometerforwardturn() {
    AutoRoutine routine = autoFactory.newRoutine("2mft"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("2mforwardturn"); //LOAD ALL PATHS HERE
    // AutoTrajectory traj2 = routine.trajectory("BLL_SL");
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    // traj1.done().onTrue(scoreL4Left());
    // traj2.done().onTrue(intake());

    return routine;
  }

  public AutoRoutine match69() {
    AutoRoutine routine = autoFactory.newRoutine("match69"); //ROUTINE NAME
    AutoTrajectory traj1 = routine.trajectory("STBM_BR"); //LOAD ALL PATHS HERE
    AutoTrajectory traj2 = routine.trajectory("BRL_SR");
    AutoTrajectory traj3 = routine.trajectory("SR_FR");

    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(()->vision.setAlignState(AlignStates.NONE)),
        superstructure.setState(SSStates.STOWED),
        traj1.resetOdometry(),

        traj1.cmd()
      )
    );

    traj1.done().onTrue(scoreL4Right().andThen(traj2.resetOdometry()).andThen(traj2.cmd()));
    traj2.done().onTrue(intake().andThen(traj3.resetOdometry()).andThen(traj3.cmd()));
    traj3.done().onTrue(scoreL4Right());

    return routine;
  }

  public Command alignLeft() {
    return choreoAlignLeft().withTimeout(alignTimeout).andThen(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING))).andThen(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2));
  }

  public Command alignRight() {
    return choreoAlignRight().withTimeout(alignTimeout).andThen(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING))).andThen(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2));
  }

  public Command scoreL2Left() {
    return 
      Commands.sequence(
        alignLeft(),
        superstructure.setState(SSStates.CORAL_2),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
  }

  public Command scoreL2Right() {
    return 
      Commands.sequence(
        alignRight(),
        superstructure.setState(SSStates.CORAL_2),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
  }

  public Command scoreL3Left() {
    return 
      Commands.sequence(
        alignLeft(),
        superstructure.setState(SSStates.CORAL_3),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
  }

  public Command scoreL3Right() {
    return 
      Commands.sequence(
        alignRight(),
        superstructure.setState(SSStates.CORAL_3),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
  }

  public Command scoreL4Left() {
    return 
      Commands.sequence(
        alignLeft(),
        superstructure.setState(SSStates.CORAL_4),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
    // return superstructure.setState(SSStates.CORAL_4).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));

  }

  public Command scoreL4Right() {
    return 
      Commands.sequence(
        alignRight(),
        superstructure.setState(SSStates.CORAL_4),
        new WaitUntilCommand(() -> elevator.atSetpoint()),
        new WaitCommand(0.1),
        superstructure.setState(SSStates.OUTTAKE).withTimeout(scoreTimeout),
        superstructure.setState(SSStates.STOWED)
      );
    // return superstructure.setState(SSStates.CORAL_4).andThen(Commands.waitSeconds(scoreTimeout)).andThen(superstructure.setState(SSStates.STOWED));

  }

  public Command intake() {
    return superstructure.setState(SSStates.INTAKE).andThen(Commands.waitSeconds(intakeTimeout)).andThen(superstructure.setState(SSStates.STOWED));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
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
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * speedMultiplier * 0.75) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * speedMultiplier * 0.75) // Drive left with negative X (left)
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

    driver.rightTrigger()
    .whileTrue(
      drivetrain.applyRequest(
        () -> new ApplyFieldSpeeds()
          .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsRight()[0], vision.getAlignOffsetsRight()[1], vision.getRotationalAlignSpeedRight()))
      ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING))))
      .onFalse(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));


    driver.rightTrigger().onFalse(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    driver.leftTrigger()
    .whileTrue(
      drivetrain.applyRequest(
        () -> new ApplyFieldSpeeds()
          .withSpeeds(new ChassisSpeeds(vision.getAlignOffsetsLeft()[0], vision.getAlignOffsetsLeft()[1], vision.getRotationalAlignSpeedLeft()))
        ).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.ALIGNING))))
    .onFalse(new InstantCommand(()->vision.setAlignState(AlignStates.NONE)));

    driver.leftTrigger().onFalse(drivetrain.applyRequest(() -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0.0, 0.0))).withTimeout(0.2).alongWith(new InstantCommand(()->vision.setAlignState(AlignStates.NONE))));

    // --------------------=Operator=--------------------

    new Trigger(operator.leftTrigger())
      .whileTrue(superstructure.setState(SSStates.INTAKE))
      .whileFalse(superstructure.setState(SSStates.STOWED));

    new Trigger(operator.rightTrigger())
      .whileTrue(superstructure.setState(SSStates.OUTTAKE))
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

    new Trigger(operator.povRight()) //.and(() -> climb.inClimbState())
      .whileTrue(superstructure.setState(SSStates.UNDOCLIMB))
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