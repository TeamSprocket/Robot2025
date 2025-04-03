package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climb.ClimbStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Outtake.OuttakeStates;
import frc.robot.subsystems.Pivot.PivotStates;

public class Superstructure extends SubsystemBase {

  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    CORAL_1,
    CORAL_2,
    CORAL_3,
    CORAL_4,
    ALGAE_REMOVE_2,
    ALGAE_REMOVE_3,
    CLIMB,
    EJECT
  }

  /**
   * funnel top roller intake - can align with 2 thingies - only need to spin wheels
   * fixed angle top and bottom outtake - to remove algae just bump into reef and spin wheels out then back away - just spin wheels for outtake
   * 28 by 28 in swerve drivebase
   * subsystems: intake (rollers mayb beam break or some sort of detection)(Emily), elevator (TBD)(Jai), shooter (rollers)(Bella), vision (Zack, Aaron, Eric), pathplanner (Jai, Bella, Emily), superstructure (Denielle)
   */

  public SSStates currentState = SSStates.NONE;

  Elevator elevator;
  Intake intake;
  Pivot pivot;
  Outtake outtake;
  Climb climb;

  private Timer timer = new Timer();

  public Superstructure(Elevator elevator, Intake intake, Outtake outtake, Pivot pivot, Climb climb) {
    timer.restart();

    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.outtake = outtake;
    this.climb = climb;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putString("SUPERSTRUCTURE STATE CURRENT", currentState.toString());
  }

  // States

  private Command stowed() {
    return Commands.runOnce(() -> {
      intake.setState(IntakeStates.STOWED);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
      climb.setState(ClimbStates.STOWED);
    });
  }

  private Command intake() {
    return Commands.sequence(
      new InstantCommand(() -> {
      outtake.setState(OuttakeStates.INTAKE);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.INTAKE);
      climb.setState(ClimbStates.STOWED);
    }),
    new WaitCommand(0.5),
    new InstantCommand(()->intake.setState(IntakeStates.INTAKE)));
  }

  private Command coral2() { // test method
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_2);
        pivot.setState(PivotStates.STOWED);
        climb.setState(ClimbStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), 
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE))
    );
  }

  private Command coral3() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_3);
        pivot.setState(PivotStates.STOWED);
        climb.setState(ClimbStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), 
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE))
    );
  }

  private Command coral4() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_4);
        pivot.setState(PivotStates.STOWED);
        climb.setState(ClimbStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()),
      new WaitCommand(0.2),
      // new InstantCommand(() -> pivot.setState(PivotStates.L4)),
      // new WaitUntilCommand(() -> pivot.atSetpoint()),
      // new WaitCommand(0.2),
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE))
    );
  }

  private Command algaeRemove2() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_2);
        pivot.setState(PivotStates.STOWED);
        climb.setState(ClimbStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), 
      new InstantCommand(() -> {outtake.setState(OuttakeStates.ALGAE_REMOVE); pivot.setState(PivotStates.ALGAE_REMOVE);})
    );
  }

  private Command algaeRemove3() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_3);
        pivot.setState(PivotStates.STOWED);
        climb.setState(ClimbStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), 
      new InstantCommand(() -> {outtake.setState(OuttakeStates.ALGAE_REMOVE); pivot.setState(PivotStates.ALGAE_REMOVE);})
    );
  }

  private Command eject() {
    return new InstantCommand(() -> {
      intake.setState(IntakeStates.EJECT);
      outtake.setState(OuttakeStates.CORAL_RECLAIM);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
      climb.setState(ClimbStates.STOWED);
    });
  }

  private Command coral1() {
    return new InstantCommand(() -> {
      intake.setState(IntakeStates.STOWED); 
      outtake.setState(OuttakeStates.CORAL1);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
      climb.setState(ClimbStates.STOWED);
    });
  }
  private Command climb() {
    return new InstantCommand(() -> {
      climb.setState(ClimbStates.CLIMB);
      intake.setState(IntakeStates.CLIMB);
      outtake.setState(OuttakeStates.STOWED);
      pivot.setState(PivotStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
    });
  }

  // ------ commands -------
  /**
   * Sets the superstructure target state
   * @param currentState Target state
   */

  public void setStowed() {
    Commands.runOnce(() -> {
      intake.setState(IntakeStates.STOWED);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
      climb.setState(ClimbStates.STOWED);
    });
  }
  
  public Command setState(SSStates wantedState) {
    switch (wantedState) {
      case NONE:
        break;

      case STOWED:
        return stowed();
        
      case INTAKE:
        return intake();
      
      case CORAL_1:
        return coral1();

      case CORAL_2:
        return coral2();
        
      case CORAL_3:
        return coral3();

      case CORAL_4:
        return coral4();

      case ALGAE_REMOVE_2:
        return algaeRemove2();
        
      case ALGAE_REMOVE_3:
        return algaeRemove3();

      case EJECT:
        return eject();
      
      case CLIMB:
        return climb();

      default:
        return Commands.print("failed");
    }
    return Commands.print("failed");
  }

  // ------ methods ------

  public SSStates getCurrentState() {
      return currentState;
  }
}