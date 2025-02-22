package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Outtake.OuttakeStates;
import frc.robot.subsystems.Pivot.PivotStates;

public class Superstructure extends SubsystemBase {

  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    CORAL_2,
    CORAL_3,
    CORAL_4,
    ALGAE_REMOVE_2,
    ALGAE_REMOVE_3,
    EJECT
  }

  /**
   * funnel top roller intake - can align with 2 thingies - only need to spin wheels
   * fixed angle top and bottom outtake - to remove algae just bump into reef and spin wheels out then back away - just spin wheels for outtake
   * 28 by 28 in swerve drivebase
   * subsystems: intake (rollers mayb beam break or some sort of detection)(Emily), elevator (TBD)(Jai), shooter (rollers)(Bella), vision (Zack, Aaron, Eric), pathplanner (Jai, Bella, Emily), superstructure (Denielle)
   */

  public SSStates currentState = SSStates.NONE;
  public SSStates lastState = SSStates.NONE;
  public SSStates wantedState = SSStates.NONE;

  Elevator elevator;
  Intake intake;
  Pivot pivot;
  Outtake outtake;

  private Timer timer = new Timer();

  public Superstructure(Elevator elevator, Intake intake, Outtake outtake, Pivot pivot) {
    timer.restart();

    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.outtake = outtake;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putString("SUPERSTRUCTURE STATE CURRENT", currentState.toString());
  }

  // Methods
  

  // ------ states ------

  // private void stowed() {
  //   intake.setState(IntakeStates.STOWED);
  //   outtake.setState(OuttakeStates.STOWED);
  //   elevator.setState(ElevatorStates.STOWED);
  //   pivot.setState(PivotStates.STOWED);
  // }

  // private void intake() {
  //   intake.setState(IntakeStates.STOWED);
  //   outtake.setState(OuttakeStates.CORAL_OUTTAKE);
  //   elevator.setState(ElevatorStates.STOWED);
  //   pivot.setState(PivotStates.STOWED);
  // }

  // private void coral2() {
  //   intake.setState(IntakeStates.STOWED);
  //   outtake.setState(OuttakeStates.STOWED);
  //   elevator.setState(ElevatorStates.CORAL_2);
  //   pivot.setState(PivotStates.STOWED);
  //   // move elevator to L2
  //   // swerve alignment might have to separate in robot container
  //   // roll outtake motors
  // }

  private Command stowed() {
    return Commands.runOnce(() -> {
      intake.setState(IntakeStates.STOWED);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
    });
  }

  private Command intake() {
    return Commands.runOnce(() -> {
      intake.setState(IntakeStates.INTAKE);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
    });
  }

  private Command coral2() { // test method
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_2);
        pivot.setState(PivotStates.STOWED);
      }).alongWith(Commands.print("saenbgfdoskadlpas")),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
  }

  private Command coral3() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_3);
        pivot.setState(PivotStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L3
    // turn pivot to correct angle
    // swerve alignment might have to separate in robot container
    // roll outtake motors
  }

  private Command coral4() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_4);
        pivot.setState(PivotStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
  }

  private Command algaeRemove2() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_2);
        pivot.setState(PivotStates.ALGAE_REMOVE);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.ALGAE_REMOVE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L2
    // pivot
    // roll wheels
  }

  private Command algaeRemove3() {
    return Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_3);
        pivot.setState(PivotStates.ALGAE_REMOVE);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.ALGAE_REMOVE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L2
    // pivot
    // roll wheels
  }

  private Command eject() {
    return new InstantCommand(() -> {
      intake.setState(IntakeStates.EJECT);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
    });
  }

  // ------ commands -------
  /**
   * Sets the superstructure target state
   * @param currentState Target state
   */
  public Command setState(SSStates wantedState) {
    switch (wantedState) {
      case NONE:
        break;

      case STOWED:
        return stowed();
        
      case INTAKE:
        return intake();
        
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