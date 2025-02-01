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
    var slot0Configs= talonFXConfigs.Slot0;
    slot0Configs.kP = 0.0 ;
    slot0Configs.kI = 0.0 ;
    slot0Configs.kD = 0.0 ;
    slot0Configs.kS = 0.0 ;
    slot0Configs.kV = 0.0 ;
    slot0Configs.kA = 0.0 ;


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
        break;

        case INTAKE:
        break;

        case HANDOFF:
        break;

        case CORAL_1:
        break;

        case CORAL_2:
        break;

        case CORAL_3:
        break;

        case ALGAE_REMOVE_2:
        break;

        case ALGAE_REMOVE_3:
        break;

        case SHALLOW_CLIMB:
        break;

        case DEEP_CLIMB: 
        break;
      }
    }




}

