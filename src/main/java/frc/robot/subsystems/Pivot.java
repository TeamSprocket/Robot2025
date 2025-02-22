package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.*;

public class Pivot extends SubsystemBase {
    MotionMagicVoltage magVelocity = new MotionMagicVoltage(0);
    private final TalonFX motor = new TalonFX(12);
    PivotStates currentState = PivotStates.NONE;

    private final SendableChooser<PivotStates> stateChooser = new SendableChooser<>();
    
    public enum PivotStates {
      NONE,
      STOWED,
      ALGAE_REMOVE
    }
  
    public Pivot(){
      motor.setPosition(0);

      stateChooser.setDefaultOption("NONE", PivotStates.NONE);
      stateChooser.addOption("STOWED", PivotStates.STOWED);
      stateChooser.addOption("ALGAE_REMOVE_2", PivotStates.ALGAE_REMOVE);
      SmartDashboard.putData("Pivot State Chooser", stateChooser);

      TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
      talonFXConfigs.withSlot0(
        new Slot0Configs()
          .withKS(0.24)//0.24
          .withKV(0.74)//
          .withKA(0)//
          .withKG(-0.42)//0.42
          .withKP(40)
          .withKI(0)
          .withKD(0)
          .withGravityType(GravityTypeValue.Arm_Cosine)
      );

      talonFXConfigs.withFeedback(
        new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.Pivot.kPivotGearRatio)
      );

      talonFXConfigs.withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(Constants.Pivot.kMotionMagicAcceleration)
        .withMotionMagicCruiseVelocity(Constants.Pivot.kMotionMagicCruiseVelocity)
      );

      motor.getConfigurator().apply(talonFXConfigs, 0.050);
      magVelocity.Slot = 0;
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("State", currentState.toString());
      // setState(stateChooser.getSelected());

      switch(currentState){
        case NONE:
        motor.setControl(magVelocity.withPosition(0));
        break;

        case STOWED:
        motor.setControl(magVelocity.withPosition(Constants.Pivot.kAngleStowed));
        break;

        case ALGAE_REMOVE:
        motor.setControl(magVelocity.withPosition(Constants.Pivot.kAngleAlgaeRemove));
        break;
      }
    }

  public void setState(PivotStates state) {
    this.currentState = state;
  }

  public Command setStateCmd() {
    return new InstantCommand(() -> setState(PivotStates.STOWED));
  }

  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.Pivot.kPivotGearRatio);

    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }
}