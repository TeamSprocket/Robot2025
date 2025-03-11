// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  MotionMagicVoltage climbVelocity = new MotionMagicVoltage(0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  
  private final TalonFX climbpivot = new TalonFX(0);
  private final TalonFX climbfollowpivot = new TalonFX(1);
  private final TalonFX algaeroll = new TalonFX(2);
}

  private States State = States.NONE;

  public enum States {
    NONE,
    STOWED,
    GET,
    GIVE,
    CLIMB
  }
  
  /** Creates a new Climb. */
  public Climb() 
  {
    
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
        .withKG(Constants.Climb.kAlgaeG)
        .withKP(Constants.Climb.kAlgaeP)
        .withKI(Constants.Climb.kAlgaeI)
        .withKD(Constants.Climb.kAlgaeD)
    );

    

    ClimbConfig.withFeedback(
        new FeedbackConfigs()
          .withSensorToMechanismRatio(Constants.Climb.kClimbGearRatio)
    );

    AlgaeConfig.withFeedback(
      new FeedbackConfigs()
      .withSensorToMechanismRatio(Constants.Climb.kAlgaeGearRatio)
    );

    climbpivot.getConfigurator().apply(ClimbConfig);
    climbpivot.setNeutralMode(NeutralModeValue.Brake);

    algaeroll.getConfigurator().apply(AlgaeConfig);
    algaeroll.setNeutralMode(NeutralModeValue.Brake);

    climbfollowpivot.setControl(new StrictFollower(climbpivot.getDeviceID()));

    

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (State)
    {
      case NONE:
        climbpivot.setControl(velocityVoltage.withVelocity(0));
        algaepivot.setControl(velocityVoltage.withVelocity(0));
        algaeroll.setControl(velocityVoltage.withVelocity(0));
      
      case STOWED:

      case GET:
      case GIVE:
      case CLIMB:

    }
  }
}
