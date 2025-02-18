package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  public static enum SSStates {
    NONE,
    STOWED,
    INTAKE,
    HANDOFF,
    CORAL_1,
    CORAL_2,
    CORAL_3,
    ALGAE_REMOVE_2,
    ALGAE_REMOVE_3,
    SHALLOW_CLIMB,
    DEEP_CLIMB
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

  private Timer timer = new Timer();

  public Superstructure() {
    timer.restart();
  }
  
  @Override
  public void periodic() {
    handleStateChange();
  }

  // Methods
  public void handleStateChange() {
    lastState = currentState;
    currentState = wantedState;

    switch (currentState) {
      case NONE:
        break;

      case STOWED:
      default:
        stowed();
        break;

      case INTAKE:
        intake();
        break;

      case HANDOFF:
        handoff();
        break;

      case CORAL_1:
        coral1();
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

      case DEEP_CLIMB:
        deepClimb();
        break;
    }
  }

  // ------ states ------

  private void stowed() {
    // all subsystems at stowed
  }

  private void intake() {
    // roll intake
    // stop it in robot container
  }

  private void handoff() {
    // roll intake
    // roll shooter
    // stop both in robot container
  }

  private void coral1() {
    // move elevator to L1
    // roll outtake motors
  }

  private void coral2() {
    // move elevator to L2
    // swerve alignment might have to separate in robot container
    // roll outtake motors
  }

  private void coral3() {
    // move elevator to L3
    // turn pivot to correct angle
    // swerve alignment might have to separate in robot container
    // roll outtake motors
  }

  private void algaeRemove2() {
    // move elevator to L2
    // pivot
    // roll wheels
  }

  private void algaeRemove3() {
    // move elevator to L2
    // pivot
    // roll wheels
  }

  private void deepClimb() {
    // ig align as best as it can
    // while true turn pivot (add hardstop maybe)
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