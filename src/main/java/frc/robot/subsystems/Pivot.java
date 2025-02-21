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

public class Pivot extends SubsystemBase {
    MotionMagicVoltage magVelocity = new MotionMagicVoltage(0);
    private final TalonFX motor_1 = new TalonFX(0); 
    PivotStates currentState = PivotStates.NONE;

    private final SendableChooser<PivotStates> stateChooser = new SendableChooser<>();
    
    public enum PivotStates {
      NONE,
      STOWED,
      ALGAE_REMOVE
    }
  
    public Pivot(){

      stateChooser.setDefaultOption("NONE", PivotStates.NONE);
      stateChooser.addOption("STOWED", PivotStates.STOWED);
      stateChooser.addOption("ALGAE_REMOVE_2", PivotStates.ALGAE_REMOVE);
      SmartDashboard.putData("Pivot State Chooser", stateChooser);

      TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
      talonFXConfigs.withSlot0(
        new Slot0Configs()
          .withKS(0.47)//0.47
          .withKV(1.1667)//1.1667
          .withKA(0.001)//0.01
          .withKG(0.4)//0.3
          .withKP(20)
          .withKI(0.0)
          .withKD(0.0)
          .withGravityType(GravityTypeValue.Arm_Cosine)
      );

      talonFXConfigs.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Pivot.kPivotGearRatio));

      MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.kMotionMagicCruiseVelocity; 
      motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.kMotionMagicAcceleration;

      motor_1.getConfigurator().apply(talonFXConfigs, 0.050);
      magVelocity.Slot = 0;
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("State", currentState.toString());

       // TODO: remove this when done tuning

      switch(currentState){
        case NONE:
        motor_1.setControl(magVelocity.withPosition(0));
        break;

        case STOWED:
        motor_1.setControl(magVelocity.withPosition(Constants.Pivot.kAngleStowed));
        break;

        case ALGAE_REMOVE:
        motor_1.setControl(magVelocity.withPosition(Constants.Pivot.kAngleAlgaeRemove));
        break;
      }
    }

  public void setState(PivotStates state) {
    this.currentState = state;
  }

  public Command setStateCmd() {
    return new InstantCommand(() -> setState(PivotStates.STOWED));
  }
}