// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
private final TalonFX motor1 = new TalonFX(14);
// private final TalonFX motor2 = new TalonFX(0);
private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public enum OuttakeStates {
    NONE,
    STOWED,
    INTAKE,
    CORAL_OUTTAKE,
    ALGAE_REMOVE
  }

  private OuttakeStates state = OuttakeStates.NONE;
  private final SendableChooser<OuttakeStates> stateChooser = new SendableChooser<>();

  private void configMotors(){

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withSlot0(
      new Slot0Configs()
      .withKP(0.0) //35
      .withKI(0)
      .withKD(0)
      .withKS(0.312) //0.312
      .withKV(0.1563) //0.1563
      .withKA(0));

      talonFXConfigs.withFeedback(
        new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.Outtake.kOuttakeGearRatio)
      );
    
    motor1.getConfigurator().apply(talonFXConfigs, 0);
    // motor2.getConfigurator().apply(talonFXConfigs, 0);
    motor1.getVelocity();
    // motor2.getVelocity(); 

  }

  /** Creates a new Outtake. */
  public Outtake() {
    stateChooser.setDefaultOption("NONE", OuttakeStates.NONE);
    stateChooser.addOption("STOWED", OuttakeStates.STOWED);
    stateChooser.addOption("CORAL_2", OuttakeStates.CORAL_OUTTAKE);
    stateChooser.addOption("ALGAE_REMOVE_2", OuttakeStates.ALGAE_REMOVE);
    SmartDashboard.putData("Elevator State Chooser", stateChooser);
    configMotors();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("OUTTALKE STATE", state.toString());
    // setState(stateChooser.getSelected()); // TODO: remove this when done testing

    switch(state){
      

      case NONE:
        motor1.setControl(velocityVoltage.withVelocity(0));
        // motor2.setControl(velocityVoltage.withVelocity(0));
        break;

      case STOWED:
        motor1.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedStowed));
        // motor2.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedStowed));
        break;

      case INTAKE:
        motor1.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedOuttake));
        // motor2.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedOuttake));
        break;

      case ALGAE_REMOVE:
        motor1.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedAlgaeRemoval));
        // motor2.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedAlgaeRemoval));
        break;

      case CORAL_OUTTAKE:
        motor1.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedOuttake));
        System.out.println("CORAL CORAL CORAL");
        // motor2.setControl(velocityVoltage.withVelocity(Constants.Outtake.kSpeedOuttake));
        break;
        
    }
    // This method will be called once per scheduler run
  }

  public void runOuttake() {
    motor1.setControl(velocityVoltage.withVelocity(0));
    // motor2.setControl(velocityVoltage.withVelocity(0));  
  }

  public void setState(OuttakeStates state) {
    this.state = state;
  }
}