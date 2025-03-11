package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  MotionMagicVoltage climbmmv = new MotionMagicVoltage(0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private final TalonFX climbpivot = new TalonFX(0);
  private final TalonFX climbfollowpivot = new TalonFX(1);
  private final TalonFX algaeroll = new TalonFX(2);

  private ClimbStates State = ClimbStates.NONE;   

  public enum ClimbStates {
    NONE,
    STOWED,
    PICKUP,
    SCORE,
    CLIMB
  }

  /** Creates a new Climb. */
  public Climb() {
    TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();
    TalonFXConfiguration AlgaeConfig = new TalonFXConfiguration();

    ClimbConfig.withSlot0(
      new Slot0Configs()
        .withKS(Constants.Climb.kClimbS)
        .withKV(Constants.Climb.kClimbV)
        .withKA(Constants.Climb.kClimbA)
        .withKG(Constants.Climb.kClimbG)
        .withKP(Constants.Climb.kClimbP)
        .withKI(Constants.Climb.kClimbI) 
        .withKD(Constants.Climb.kClimbD)
    );

    AlgaeConfig.withSlot0(
      new Slot0Configs()
        .withKS(Constants.Climb.kAlgaeS)
        .withKV(Constants.Climb.kAlgaeV)
        .withKA(Constants.Climb.kAlgaeA)
        .withKP(Constants.Climb.kAlgaeP)
        .withKI(Constants.Climb.kAlgaeI)
        .withKD(Constants.Climb.kAlgaeD)
    );

    ClimbConfig.withFeedback(
        new FeedbackConfigs()
          .withSensorToMechanismRatio(Constants.Climb.kClimbGearRatio)
    );

    ClimbConfig.withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(Constants.Climb.kMotionMagicAcceleration)
        .withMotionMagicCruiseVelocity(Constants.Climb.kMotionMagicCruiseVelocity)
      );

    AlgaeConfig.withFeedback(
      new FeedbackConfigs()
      .withSensorToMechanismRatio(Constants.Climb.kAlgaeGearRatio)
    );

    climbpivot.getConfigurator().apply(ClimbConfig);
    climbpivot.setNeutralMode(NeutralModeValue.Brake);

    algaeroll.getConfigurator().apply(AlgaeConfig);
    algaeroll.setNeutralMode(NeutralModeValue.Brake);

    climbfollowpivot.getConfigurator().apply(
      new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
    );

    climbfollowpivot.setControl(new StrictFollower(climbpivot.getDeviceID()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (State) {
      case NONE:
        climbpivot.setControl(climbmmv.withPosition(0.0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
        break;

      case STOWED:
        climbpivot.setControl(climbmmv.withPosition(0.0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
        break;

      case PICKUP:
        climbpivot.setControl(climbmmv.withPosition(0.0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
        break;

      case SCORE:
        climbpivot.setControl(climbmmv.withPosition(0.0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
        break;

      case CLIMB:
        climbpivot.setControl(climbmmv.withPosition(0.0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
        break;
    }
  }

  public void setState(ClimbStates state) {
    this.State = state;
  }
}
