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

    public static final Pose2d poseAlignBlueLeft17 = new Pose2d(3.84, 3.28, Rotation2d.fromDegrees(60));
    public static final Pose2d poseAlignBlueRight17 = new Pose2d(4.12, 3.07, Rotation2d.fromDegrees(60));
    public static final Pose2d poseAlignBlueLeft18 = new Pose2d(3.5, 4.23, Rotation2d.fromDegrees(0)); //4.25
    public static final Pose2d poseAlignBlueRight18 = new Pose2d(3.55, 3.88, Rotation2d.fromDegrees(0)); //3.90
    public static final Pose2d poseAlignBlueLeft19 = new Pose2d(4.15, 4.95, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignBlueRight19 = new Pose2d(3.85, 4.75, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignBlueLeft20 = new Pose2d(5.15, 4.77, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignBlueRight20 = new Pose2d(4.82, 4.95, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignBlueLeft21 = new Pose2d(5.484, 3.85, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignBlueRight21 = new Pose2d(5.491, 4.21, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignBlueLeft22 = new Pose2d(4.84, 3.01, Rotation2d.fromDegrees(120));
    public static final Pose2d poseAlignBlueRight22 = new Pose2d(5.17, 3.18, Rotation2d.fromDegrees(120));

    public static final Pose2d poseAlignRedLeft10 = new Pose2d(12.157, 4.26, Rotation2d.fromDegrees(0));
    public static final Pose2d poseAlignRedRight10 = new Pose2d(12.157, 3.8, Rotation2d.fromDegrees(0));
    public static final Pose2d poseAlignRedLeft9 = new Pose2d(12.774, 4.761, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignRedRight9 = new Pose2d(12.519, 4.691, Rotation2d.fromDegrees(300));
    public static final Pose2d poseAlignRedLeft8 = new Pose2d(13.635, 4.750, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignRedRight8 = new Pose2d(13.425, 4.980, Rotation2d.fromDegrees(240));
    public static final Pose2d poseAlignRedLeft7 = new Pose2d(14.025, 3.882, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignRedRight7 = new Pose2d(14.025, 4.190, Rotation2d.fromDegrees(180));
    public static final Pose2d poseAlignRedLeft6 = new Pose2d(13.398, 3.089, Rotation2d.fromDegrees(120));
    public static final Pose2d poseAlignRedRight6 = new Pose2d(13.680, 3.250, Rotation2d.fromDegrees(120));
    public static final Pose2d poseAlignRedLeft11 = new Pose2d(12.359, 3.200, Rotation2d.fromDegrees(60));
    public static final Pose2d poseAlignRedRight11 = new Pose2d(12.684, 3.078, Rotation2d.fromDegrees(60));
  }

  public static final class Intake {
    
    public static final double kSpeedIntake = -20;//-15
    public static final double kSpeedStowed = 0;
    public static final double kSpeedEject = 7;

    public static final double kIntakeS = 0.3;
    public static final double kIntakeV = 0.17; //0.132
    public static final double kIntakeA = 0.0;  
    public static final double kIntakeG = 0.0;
    public static final double kIntakeP = 0.0;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
    public static final double kIntakeGearRatio = 1.0;
  }

  public static final class Outtake {
    
    public static final double kSpeedStowed = 0.0;
    public static final double kSpeedOuttake = 9;
    public static final double kSpeedAlgaeRemoval = -6.5;
    public static final double kSpeedIntake = 4;

    public static final double kTuneSpeed = 0.3;

    public static final double kOuttakeS = 0.25; // 0.25
    public static final double kOuttakeV = 0.2083; // 0.153
    public static final double kOuttakeA = 0.0; //nothing
    public static final double kOuttakeP = 0.0;
    public static final double kOuttakeI = 0.0;
    public static final double kOuttakeD = 0.0;
    public static final double kOuttakeGearRatio = 1.0;
  }

  public static final class Pivot {
    
    public static final double kAngleStowed = -0.14; //-0.22 -0.19
    public static final double kAngleAlgaeRemove = 0.04;
    public static final double kAngleIntake = 0.224; //0.105
    public static final double kAngleL4 = 0.06;

    public static final double kMotionMagicCruiseVelocity = 6;//2
    public static final double kMotionMagicAcceleration = 7; //4
    public static final double kP = 8; //50
    public static final double kI = 0.0;
    public static final double kD = 1.2;
    public static final double kS = 0.16; //0.24 0.45 
    public static final double kV = 0.5; // //0.5 //0.74 0.617
    public static final double kA = 0.0;
    public static final double kG = -0.45; // -0.45 //-0.42 0.45

    public static final double kPivotGearRatio = 2.75;
  }

  public static class Elevator {
    public static final double kMAX_HEIGHT_UP = 6.5;

    public static final double kHeightStowed = 0.0;
    public static final double kHeightIntake = 0.0;
    public static final double kHeightHandoff = 0.0;
    public static final double kHeightCoral1 = 0.0;
    public static final double kHeightCoral2 = 1.63; //1.65
    public static final double kHeightCoral3 = 3.32; //3.4
    public static final double kHeightCoral4 = 5.85; //5.8

    public static final double kHeightAlgaeRemove2 = 1.1; //1
    public static final double kHeightAlgaeRemove3 = 2.8; //2.7
    public static final double kHeightShallowClimb = 0.0;
    public static final double kHeightDeepClimb = 0.0; 

    public static final double kMotionMagicCruiseVelocity = 25.0; //22
    public static final double kMotionMagicAcceleration = 25.0; //24

    public static final double kP = 65.0; // 40.0
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.1375; //0.105
    public static final double kV = 0.769; //0.738
    public static final double kA = 0.001; //0.001
    public static final double kG = 0.3125; //0.375

    public static final double kElevatorGearRatio = 5.77;
  }

  public static class Climb{
    public static final double kClimbStowed = 0.0;
    public static final double kClimbPosition = 472; 
  }
}
