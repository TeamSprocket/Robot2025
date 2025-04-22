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
    public static final double kLimelightHeightMeters = 0;

    public static final double kAcceptableVolatilityThreshold = 0.2;

    public static final double kMaxDrivingSpeed = 0.0;
    public static final double kMaxTurningSpeed = 0.1;

    public static final int kVolatilitySlidingWindowLen = 20;

    public static final double kdistanceOffset = 0.7642;
    
    public static final Pose2d Blue17 = new Pose2d(3.9883315563201904 , 3.1964995861053467, Rotation2d.fromDegrees(60));
    public static final Pose2d Blue18 = new Pose2d(3.5167877674102783 , 4.027209281921387, Rotation2d.fromDegrees(0));
    public static final Pose2d Blue19 = new Pose2d(4.010903358459473 , 4.878874778747559, Rotation2d.fromDegrees(300));
    public static final Pose2d Blue20 = new Pose2d(4.992037296295166 , 4.88698148727417 , Rotation2d.fromDegrees(240));
    public static final Pose2d Blue21 = new Pose2d(5.485454082489014 , 4.010403861999512 , Rotation2d.fromDegrees(180));
    public static final Pose2d Blue22 = new Pose2d(4.977437496185303 , 3.175410270690918 , Rotation2d.fromDegrees(120)); 

    public static final Pose2d Red6 = new Pose2d(13.551526069641113 , 3.166419267654419 , Rotation2d.fromDegrees(120));
    public static final Pose2d Red7 = new Pose2d(14.050718307495117 , 4.028757572174072 , Rotation2d.fromDegrees(180));
    public static final Pose2d Red8 = new Pose2d(13.561223983764648 , 4.886585712432861 , Rotation2d.fromDegrees(240));
    public static final Pose2d Red9 = new Pose2d(12.576774597167969  , 4.876892566680908 , Rotation2d.fromDegrees(300));
    public static final Pose2d Red10 = new Pose2d(12.080918312072754 , 4.028757572174072 , Rotation2d.fromDegrees(0));
    public static final Pose2d Red11 = new Pose2d(12.585830459594726 , 3.174359083175659  , Rotation2d.fromDegrees(60));

    public static final double xOffset = 0.172;

    public static final double kFFAlignSpeed = 2.5;
  }

  public static final class Intake {
    
    public static final double kSpeedIntake = -15;//-15
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
    public static final double kSpeedOuttake = 5.85;
    public static final double kSpeedOuttakeL4 = 9;
    public static final double kSpeedAlgaeRemoval = -6.5;
    public static final double kSpeedIntake = 6;

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
    
    public static final double kAngleStowed = -0.2746; //-0.22 -0.19
    public static final double kAngleAlgaeRemove = -0.05;
    public static final double kAngleIntake = 0.30; //0.224
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
    public static final double kHeightCoral2 = 1.52; //1.65
    public static final double kHeightCoral3 = 3.15; //3.4
    public static final double kHeightCoral4 = 5.775; //5.8

    public static final double kHeightAlgaeRemove2 = 1.1; //1
    public static final double kHeightAlgaeRemove3 = 2.8; //2.7
    public static final double kHeightShallowClimb = 0.0;

    public static final double kHeightDeepClimb = 0.0; 

    public static final double kMotionMagicCruiseVelocity = 25.0; //25
    public static final double kMotionMagicAcceleration = 25.0; //25

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