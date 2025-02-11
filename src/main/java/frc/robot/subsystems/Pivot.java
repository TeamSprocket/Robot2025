package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    MotionMagicVoltage magVelocity = new MotionMagicVoltage(0);
    private final TalonFX motor_1 = new TalonFX(0); 
    
  
    public Pivot(){
      var talonFXConfigs = new TalonFXConfiguration();
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


      var motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 5; 
      motionMagicConfigs.MotionMagicAcceleration = 160;


      motor_1.getConfigurator().apply(talonFXConfigs, 0.050);
      magVelocity.Slot = 0;
    }
    
    

    PivotStates currentState = PivotStates.NONE;

    public enum PivotStates {
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
        DEEP_CLIMB,
      }{
      switch(currentState){
        case NONE:
        motor_1.setControl(magVelocity.withPosition(0));
        break;

        case STOWED:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case INTAKE:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case HANDOFF:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case CORAL_1:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case CORAL_2:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case CORAL_3:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case ALGAE_REMOVE_2:
        motor_1.setControl(magVelocity.withPosition(4));

        break;

        case ALGAE_REMOVE_3:
        motor_1.setControl(magVelocity.withPosition(4));

        break;

        case SHALLOW_CLIMB:
        motor_1.setControl(magVelocity.withPosition(0));

        break;

        case DEEP_CLIMB: 
        motor_1.setControl(magVelocity.withPosition(0));

        break;
      }
    }




}

