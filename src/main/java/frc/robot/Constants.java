// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class Superstructure {

  }

  public static class Elevator {
    public static final double kMAX_HEIGHT_UP = 4.5; //5.0 in reality
    public static final double kHeightStowed = 0.0;
    public static final double kHeightCoral2 = 1.0;
    public static final double kHeightCoral3 = 4.0;
    public static final double kHeightAlgaeRemove2 = 0.0;
    public static final double kHeightAlgaeRemove3 = 0.0;

    public static final double kMotionMagicCruiseVelocity = 4.0;
    public static final double kMotionMagicAcceleration = 8.0;

    public static final double kP = 40;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.105;
    public static final double kV = 0.738;
    public static final double kA = 0.001;
    public static final double kG = 0.375;

    public static final double kElevatorGearRatio = 5.77;
  }
}