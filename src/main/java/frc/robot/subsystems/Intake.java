// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  final TalonFX intakemotor = new TalonFX(0);
  VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private IntakeStates state = IntakeStates.NONE;

  public enum IntakeStates {
    NONE,
    STOWED,
    HANDOFF,
    CORAL_1,
    CORAL_2,
    CORAL_3,
    ALGAE_REMOVE_2,
    ALGAE_REMOVE_3,
    SHALLOW_CLIMB,
    DEEP_CLIMB
  }

  /** Creates a new Intake. */
  public Intake() 
  {
    TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();
    IntakeConfig.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(Constants.Intake.kIntakeS) // 0.27
                .withKV(Constants.Intake.kIntakeV) // 1.4
                .withKA(Constants.Intake.kIntakeA) // 0.01
                .withKG(Constants.Intake.kIntakeG) // 0.20
    );
  }
  
    @Override
    public void periodic() {

    switch (state) {
        case NONE:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case STOWED:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;
            
        case HANDOFF:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case CORAL_1:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case CORAL_2:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case CORAL_3:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case ALGAE_REMOVE_2:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case ALGAE_REMOVE_3:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case SHALLOW_CLIMB:
          intakemotor.setControl(velocityVoltage.withVelocity(0));
          break;

        case DEEP_CLIMB:
          intakemotor.setControl(velocityVoltage);
          break;
  
    // This method will be called once per scheduler run
  }
}
}
