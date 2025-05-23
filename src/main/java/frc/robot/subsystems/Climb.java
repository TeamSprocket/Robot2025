package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climb;
import frc.util.Util;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  MotionMagicVoltage climbmmv = new MotionMagicVoltage(0);

  private final TalonFX climbpivot = new TalonFX(RobotMap.Climb.CLIMB_PIVOT);


  private ClimbStates State = ClimbStates.NONE;    

  public enum ClimbStates {
    NONE,
    STOWED,
    CLIMB,
    UNDOCLIMB
  }

  /** Creates a new Climb. */
  public Climb() {
    TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();
    climbpivot.setPosition(0);

    climbpivot.getConfigurator().apply(ClimbConfig);
    climbpivot.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    switch (State) {
      case NONE:
        climbpivot.set(0);
        break;

      case STOWED:
        climbpivot.setVoltage(0);
        break;

      case CLIMB:
        climbpivot.setVoltage(7);//-2 //2
        break;

      case UNDOCLIMB:
        climbpivot.setVoltage(-7);
        break;
    }
  }

  public void setState(ClimbStates state) {
    this.State = state;
  }
}