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
    // public static final Pose2d testPose = new Pose2d(4, 0, Rotation2d.fromDegrees(0));
    


    // Most likely can be same blue/red
    // public static final Translation3d targetPoint = new Translation3d(-0.1, (8.211 - 5.548), 0);

    public static final double kShooterVelocityLinearMultiplier = 1.5;
    public static final double kShooterVelocityBase = 17.0;
    
    /**
     * @param dist Distance from bot shooter to targetPoint in meters
     * @return Array of 2 values: shooterPivot angle in degrees, shooter velocity in MPS
     */
    // public static double[] getValues(double dist) {
    //     int index = (int) ((dist - 0.5) * 100);
    //     double[] retArr = {vals[index][1], vals[index][2]};
    //     return retArr;
    // }

    // VALS HERE
    // distance in meters of april tag, angle from 0-90 (0 is horiz), shooter mps
}
}
