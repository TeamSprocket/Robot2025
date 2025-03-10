// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

/** Add your docs here. */
public class Climb {





    public TalonFX climbMotor = new TalonFX(11);
public TalonFX climbFollowMotor = new TalonFX(19);

  public Climb(){
    climbFollowMotor.setControl(new StrictFollower(climbMotor.getDeviceID()));
    
    climbMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    climbFollowMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
  };

public void runMotor(){
climbMotor.set(0.1);

}
public void stopMotor(){
  climbMotor.set(0);
}


}
