package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0.0)
          .withKG(0.0)
          .withKP(0.0)
          .withKI(0.0)
          .withKD(0.0)
      );

      MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Pivot.kMotionMagicCruiseVelocity; 
      motionMagicConfigs.MotionMagicAcceleration = Constants.Pivot.kMotionMagicAcceleration;

      motor_1.getConfigurator().apply(talonFXConfigs, 0.050);
      magVelocity.Slot = 0;
    }

    @Override
    public void periodic() {

      setState(stateChooser.getSelected()); // TODO: remove this when done tuning

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
}