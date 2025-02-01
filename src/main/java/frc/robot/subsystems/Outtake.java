// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
private final TalonFX motor1 = new TalonFX(0);
private final TalonFX motor2 = new TalonFX(0);



  public enum OuttakeStates {
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
  private OuttakeStates state = OuttakeStates.NONE;

  private void configMotors(){
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    talonFXConfigs.withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0));
    motor1.getConfigurator().apply(talonFXConfigs, 0);
    motor2.getConfigurator().apply(talonFXConfigs, 0);

    m_velocity.Slot = 0;
    motor1.setControl(m_velocity);
    motor2.setControl(m_velocity);
  }

  /** Creates a new Outtake. */
  public Outtake() {
  }

  @Override
  public void periodic() {
    VelocityVoltage stopControl = new VelocityVoltage(0);
    switch(state){
      case NONE:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;

      case STOWED:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;

      case INTAKE:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
    
      case HANDOFF:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
    
      case CORAL_1:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;

      case CORAL_2:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
      
      case CORAL_3:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;

      case ALGAE_REMOVE_2:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;

      case ALGAE_REMOVE_3:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
      
      case SHALLOW_CLIMB:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
      
      case DEEP_CLIMB:
        motor1.setControl(stopControl);
        motor2.setControl(stopControl);
        break;
        
    }
    // This method will be called once per scheduler run
  }
}
