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
    ALGAE_REMOVE_2,
    ALGAE_REMOVE_3
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
    handleStateChange();
    SmartDashboard.putString("SUPERSTRUCTURE STATE CURRENT", currentState.toString());
  }

  // Methods
  public void handleStateChange() {
    lastState = currentState;
    currentState = wantedState;

    switch (currentState) {
      case NONE:
        break;

      case STOWED:
        stowed();
        break;

      case INTAKE:
        intake();
        break;

      case CORAL_2:
        coral2();
        break;

      case CORAL_3:
        coral3();
        break;

      case ALGAE_REMOVE_2:
        algaeRemove2();
        break;

      case ALGAE_REMOVE_3:
        algaeRemove3();
        break;
    }
  }

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

  private void stowed() {
    Commands.runOnce(() -> {
      intake.setState(IntakeStates.STOWED);
      outtake.setState(OuttakeStates.STOWED);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
    });
  }

  private void intake() {
    Commands.runOnce(() -> {
      intake.setState(IntakeStates.STOWED);
      outtake.setState(OuttakeStates.CORAL_OUTTAKE);
      elevator.setState(ElevatorStates.STOWED);
      pivot.setState(PivotStates.STOWED);
    });
  }

  private void coral2() { // test method
    System.out.println("TESTESTETSTETSSTETSTST");
    Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_2);
        pivot.setState(PivotStates.STOWED);
      }).alongWith(Commands.print("saenbgfdoskadlpas")),
      new WaitCommand(1), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
  }

  private void coral3() {
    Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.CORAL_3);
        pivot.setState(PivotStates.STOWED);
      }),
      new WaitCommand(1), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.CORAL_OUTTAKE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L3
    // turn pivot to correct angle
    // swerve alignment might have to separate in robot container
    // roll outtake motors
  }

  private void algaeRemove2() {
    Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_2);
        pivot.setState(PivotStates.STOWED);
      }),
      new WaitUntilCommand(() -> elevator.atSetpoint()), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.ALGAE_REMOVE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L2
    // pivot
    // roll wheels
  }

  private void algaeRemove3() {
    Commands.sequence(
      new InstantCommand(() -> {
        intake.setState(IntakeStates.STOWED); 
        outtake.setState(OuttakeStates.STOWED);
        elevator.setState(ElevatorStates.ALGAE_REMOVE_3);
        pivot.setState(PivotStates.STOWED);
      }),
      new WaitCommand(1), // wait until elevator goes up (might not need or use function to detect)
      new InstantCommand(() -> outtake.setState(OuttakeStates.ALGAE_REMOVE)).alongWith(Commands.print("OUTTAKEING"))
      // .andThen(new WaitCommand(1))
    );
    // move elevator to L2
    // pivot
    // roll wheels
  }

  // ------ commands -------
  /**
   * Sets the superstructure target state
   * @param currentState Target state
   */
  public Command setState(SSStates wantedState) {
      return new InstantCommand(() -> this.wantedState = wantedState);
  }

  // ------ methods ------

  public SSStates getCurrentState() {
      return currentState;
  }
}