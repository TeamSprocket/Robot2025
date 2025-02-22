// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

  public static class Superstructure {
    public static double kAtGoalTolerance = 0.2;
  }

  public static final class Vision {

    public static final Matrix<N3, N1> kMultipleTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

    // measurements
    public static final Translation3d targetPointBlue = new Translation3d(0.0, 5.448, 0); // 5.548  // Moved towards right red alliance 0.1m
    public static final Translation3d targetPointRed = new Translation3d(0.0, 2.563, 0); //2.663
    
    public static final double kLimelightPitchAngleDegrees = 5.0;
    public static final double kLimelightHeightMeters = 0; // TODO

    public static final double kAcceptableVolatilityThreshold = 0.2;

    public static final double kMaxDrivingSpeed = 0.0;
    public static final double kMaxTurningSpeed = 0.1;

    public static final int kVolatilitySlidingWindowLen = 20;

    public static final double kdistanceOffset = 0.7642; // 0.5992

    public static final Pose2d poseAlignBlueLeft17 = new Pose2d(3.56, 2.82, Rotation2d.fromDegrees(60));
    public static final Pose2d poseAlignBlueRight17 = new Pose2d(3.83, 2.635, Rotation2d.fromDegrees(60));
    public static final Pose2d poseAlignBlueLeft18 = new Pose2d(3.0, 4.2, Rotation2d.fromDegrees(0));
    public static final Pose2d poseAlignBlueRight18 = new Pose2d(3.0, 3.85, Rotation2d.fromDegrees(0));
    public static final Pose2d poseAlignBlueLeft19 = new Pose2d(3.9, 5.4, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignBlueRight19 = new Pose2d(3.6, 5.25, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignBlueLeft20 = new Pose2d(5.35, 5.235, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignBlueRight20 = new Pose2d(5.05, 5.38, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignBlueLeft21 = new Pose2d(6.0, 3.85, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignBlueRight21 = new Pose2d(6.0, 4.2, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignBlueLeft22 = new Pose2d(5.125, 2.675, Rotation2d.fromDegrees(120));
    public static final Pose2d poseAlignBlueRight22 = new Pose2d(5.45, 2.86, Rotation2d.fromDegrees(120));
  }

  public static final class Intake {
    
    public static final double kSpeedIntake = 20;
    public static final double kSpeedStowed = 0;
    public static final double kSpeedEject = -5;

    public static final double kIntakeS = 0.182;
    public static final double kIntakeV = 0.132;
    public static final double kIntakeA = 0.0;  
    public static final double kIntakeG = 0.0;
    public static final double kIntakeP = 0.0;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
    public static final double kIntakeGearRatio = 1.0;
  }

  public static final class Outtake {
    
    public static final double kSpeedStowed = 0.0;
    public static final double kSpeedOuttake = 6.0;
    public static final double kSpeedAlgaeRemoval = -7.5;

    public static final double kOuttakeS = 0.25; // 0.25
    public static final double kOuttakeV = 0.0; // 0.153
    public static final double kOuttakeA = 0.0; //nothing
    public static final double kOuttakeG = 0.0; //nothing
    public static final double kOuttakeP = 0.0;
    public static final double kOuttakeI = 0.0;
    public static final double kOuttakeD = 0.0;
    public static final double kOuttakeGearRatio = 1.0;
  }

  public static final class Pivot {
    
    public static final double kAngleStowed = 0.2;
    public static final double kAngleAlgaeRemove = 0.0;

    public static final double kMotionMagicCruiseVelocity = 2;
    public static final double kMotionMagicAcceleration = 4;

    public static final double kP = 40;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.24;
    public static final double kV = 0.74;
    public static final double kA = 0.0;
    public static final double kG = -0.42;

    public static final double kPivotGearRatio = 2.75;
  }

  public static class Elevator {
    public static final double kMAX_HEIGHT_UP = 4.5;

    public static final double kHeightStowed = 0.0;
    public static final double kHeightIntake = 0.0;
    public static final double kHeightHandoff = 0.0;
    public static final double kHeightCoral1 = 0.0;
    public static final double kHeightCoral2 = 1.0;
    public static final double kHeightCoral3 = 4.0;
    public static final double kHeightAlgaeRemove2 = 3.0;
    public static final double kHeightAlgaeRemove3 = 0.0;
    public static final double kHeightShallowClimb = 0.0;
    public static final double kHeightDeepClimb = 0.0;

    public static final double kMotionMagicCruiseVelocity = 4.0;
    public static final double kMotionMagicAcceleration = 8.0;

    public static final double kP = 40.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.105;
    public static final double kV = 0.738;
    public static final double kA = 0.001;
    public static final double kG = 0.375;

    public static final double kElevatorGearRatio = 5.77;
  }
}
