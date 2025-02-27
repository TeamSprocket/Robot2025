package frc.robot.subsystems;

import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;
import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.LimelightHelper;
import frc.util.ShuffleboardIO;
import frc.util.Util;

public class Vision extends SubsystemBase {
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Current Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Testing Pose", Pose2d.struct).publish();

    Timer timer = new Timer();

    Pose2d testPose = new Pose2d();

    public enum AlignStates{
        ALIGNING,
        NONE
    }

    AlignStates currentAlignState = AlignStates.NONE;

    Alliance allianceColor = Alliance.Blue;

    public boolean updatePose = false;

    private int[] blueReefAprilTag = {17, 18, 19, 20, 21, 22};
    private int[] redReefAprilTag = {6, 7, 8, 9, 10, 11};

    CommandSwerveDrivetrain drivetrain;

    String name = "limelight-front";
    int counter = 0;

    Pose2d lastPose = new Pose2d();
    double lastTimeStamp = 0.0;

    LimelightHelper.PoseEstimate estimate;

    Pose2d endpointL = new Pose2d();
    Pose2d endpointR = new Pose2d();

    Command pathL;
    Command pathR;

    double distToAprilLeft = 0.0;
    double distToAprilRight = 0.0;
    boolean updateFirst = true;

    double fiducialID;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
        timer.reset();
        timer.start();

        
        ShuffleboardIO.addSlider("Alignment X", 0, 7, 0);
        ShuffleboardIO.addSlider("Alignment Y", 0, 7, 0);
    }

    @Override
    public void periodic() {
        testPose = getPoseTesting();

        if (timer.get() > 0.2 && currentAlignState == AlignStates.ALIGNING) {
            updateAlignPose();
            // System.out.println("UPDATING");
            timer.reset();
            timer.start();
        }
        
        debug();
    }

    public void setAlignState(AlignStates alignState) {
        currentAlignState = alignState;
    }

    public Pose2d getPoseLeft() {
        fiducialID = getTargetTagLeft();
        endpointL = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpointL = Constants.Vision.poseAlignBlueLeft17;
                break;
            case 18:
                endpointL = Constants.Vision.poseAlignBlueLeft18;
                break;
            case 19:
                endpointL = Constants.Vision.poseAlignBlueLeft19;
                break;
            case 20:
                endpointL = Constants.Vision.poseAlignBlueLeft20;
                break;
            case 21:
                endpointL = Constants.Vision.poseAlignBlueLeft21;
                break;
            case 22:
                endpointL = Constants.Vision.poseAlignBlueLeft22;
                break;
            case -1:
                endpointL = drivetrain.getAutoBuilderPose();
                break;
        }

        return endpointL;
    }
    
    public Pose2d getPoseRight() {
        fiducialID = getTargetTagRight();
        endpointR = new Pose2d();
        switch ((int)fiducialID) {
            case 17:
                endpointR = Constants.Vision.poseAlignBlueRight17;
                break;
            case 18:
                endpointR = Constants.Vision.poseAlignBlueRight18;
                break;
            case 19:
                endpointR = Constants.Vision.poseAlignBlueRight19;
                break;
            case 20:
                endpointR = Constants.Vision.poseAlignBlueRight20;
                break;
            case 21:
                endpointR = Constants.Vision.poseAlignBlueRight21;
                break;
            case 22:
                endpointR = Constants.Vision.poseAlignBlueRight22;
                break;
            case -1:
                endpointR = drivetrain.getAutoBuilderPose();
                break;
        }

        // AutoBuilder.pathfindToPose(
        //     endpointL,
        //     new PathConstraints(2, 2, 3, 2), 0.0
        // );

        return endpointR;
    }

    public Pose2d getPoseTesting() {
        testPose = new Pose2d(
            // ShuffleboardIO.getDouble("Alignment X"), 
            // ShuffleboardIO.getDouble("Alignment Y"), 
            // Rotation2d.fromDegrees(180));
            12.684, 3.078,Rotation2d.fromDegrees(60));
        return testPose;


    }

    private int getTargetTagLeft() {
      int tag = -1;
      double minDistance = Integer.MAX_VALUE;
      for (int i = 6; i <= 11; i++) {
        Pose2d target = new Pose2d(1000, 1000, new Rotation2d(0));
        if (i == 6) target = Constants.Vision.poseAlignRedLeft6;
        else if (i == 7) target = Constants.Vision.poseAlignRedLeft7;
        else if (i == 8) target = Constants.Vision.poseAlignRedLeft8;
        else if (i== 9) target = Constants.Vision.poseAlignRedLeft9;
        else if (i == 10) target = Constants.Vision.poseAlignRedLeft10;
        else if (i == 11) target = Constants.Vision.poseAlignRedLeft11;

        double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
        if (distance < minDistance) {
            minDistance = distance;
            tag = i;
        }
      }

      for (int i = 17; i <= 22; i++) {
        Pose2d target = new Pose2d(1000, 1000, new Rotation2d(0));
        if (i == 17) target = Constants.Vision.poseAlignBlueLeft17;
        else if (i == 18) target = Constants.Vision.poseAlignBlueLeft18;
        else if (i == 19) target = Constants.Vision.poseAlignBlueLeft19;
        else if (i == 20) target = Constants.Vision.poseAlignBlueLeft20;
        else if (i == 21) target = Constants.Vision.poseAlignBlueLeft21;
        else if (i == 22) target = Constants.Vision.poseAlignBlueLeft22;

        double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
        if (distance < minDistance) {
            minDistance = distance;
            tag = i;
        }
      }

      return tag;
    }

    private int getTargetTagRight() {
        int tag = -1;
        double minDistance = Integer.MAX_VALUE;
        for (int i = 6; i <= 11; i++) {
          Pose2d target = new Pose2d(1000, 1000, new Rotation2d(0));
          if (i == 6) target = Constants.Vision.poseAlignRedRight6;
          else if (i == 7) target = Constants.Vision.poseAlignRedRight7;
          else if (i == 8) target = Constants.Vision.poseAlignRedRight8;
          else if (i== 9) target = Constants.Vision.poseAlignRedRight9;
          else if (i == 10) target = Constants.Vision.poseAlignRedRight10;
          else if (i == 11) target = Constants.Vision.poseAlignRedRight11;
  
          double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
          if (distance < minDistance) {
              minDistance = distance;
              tag = i;
          }
        }
  
        for (int i = 17; i <= 22; i++) {
          Pose2d target = new Pose2d(1000, 1000, new Rotation2d(0));
          if (i == 17) target = Constants.Vision.poseAlignBlueRight17;
          else if (i == 18) target = Constants.Vision.poseAlignBlueRight18;
          else if (i == 19) target = Constants.Vision.poseAlignBlueRight19;
          else if (i == 20) target = Constants.Vision.poseAlignBlueRight20;
          else if (i == 21) target = Constants.Vision.poseAlignBlueRight21;
          else if (i == 22) target = Constants.Vision.poseAlignBlueRight22;
  
          double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
          if (distance < minDistance) {
              minDistance = distance;
              tag = i;
          }
        }

        return tag;
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    public Translation2d getTranslation2d() {
        LimelightHelper.PoseEstimate estimate;
        if (LimelightHelper.getTV(name)) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            }
            else {
                estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2(name);
            }
            return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
        } else {
            return new Translation2d(0.0, 0.0);
        }
    }

    public Pose2d getPose2d() {
        if (LimelightHelper.getTV(name)) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            lastPose = estimate.pose;
            return estimate.pose;
        } else {
            return lastPose;
        }
    }

    public double getTX() {
        if (hasTargets()) {
            return LimelightHelper.getTX(name);
        } else {
            return 0.0;
        }
    }

    // public void updateAlignPose() {
    //     if (LimelightHelper.getTV(name) && updatePose) {
    //         estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
    //         drivetrain.resetPose(estimate.pose);
    //     }
    // }

    public void updateAlignPose() {
        if (LimelightHelper.getTV(name)) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            drivetrain.resetPose(estimate.pose);
        }
    }

    public double getTimeStamp() {
        if (LimelightHelper.getTV(name)) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            lastTimeStamp = estimate.timestampSeconds;
            return estimate.timestampSeconds;
        } else {
            return lastTimeStamp;
        }
    }

    public double getDistToTarget() {
        if (LimelightHelper.getTV(name)) {
            double dist = (0.3048 - Constants.Vision.kLimelightHeightMeters) / Math.tan(LimelightHelper.getTY(name)); // tan(vertical angle) = height of robot to apriltag / distance to april tag
            return dist;
        } else {
            return 1.5; // if there is no target return a value greater than 1 so pose doesn't update
        }
    }

    public boolean hasTargets() {
        return LimelightHelper.getTV(name);
    }
     
    public boolean hasReefTargets(){
        if(LimelightHelper.getTV(name) ){
            for (int reefID : redReefAprilTag) {
                if (LimelightHelper.getFiducialID(name) == reefID) return true;
            }
            for (int reefID : blueReefAprilTag) {
                if (LimelightHelper.getFiducialID(name) == reefID) return true;
            }
        } 
        return false;
    }

    // public Pose2d addVisionPose() {
    //     if (hasTargets()) {
    //         Pose2d pose = getPose2d();
    //         drivetrain.addVisionMeasurement(pose, getTimeStamp());
    //         return pose;
    //     } else {
    //         return drivetrain.getAutoBuilderPose();
    //     }
    // }

    public Pose2d resetPose() {
        if (hasTargets()) {
            Pose2d pose = getPose2d();
            drivetrain.resetTeleopPose(pose);
            return pose;
        } else {
            return drivetrain.getAutoBuilderPose();
        }
    }
    
    private void debug() {
        SmartDashboard.putBoolean("Has Reef Target [VI]", hasReefTargets());
        SmartDashboard.putNumber("APRILTAG POSE", getPose2d().getX());
        SmartDashboard.putNumber("END XL",endpointL.getX());
        SmartDashboard.putNumber("END YL",endpointL.getY());
        SmartDashboard.putNumber("END XR",endpointR.getX());
        SmartDashboard.putNumber("END YR",endpointR.getY());
        SmartDashboard.putNumber("dist to left", distToAprilLeft);
        SmartDashboard.putNumber("dist to right", distToAprilRight);
        SmartDashboard.putNumber("times reset", counter);
        SmartDashboard.putBoolean("has targets", hasTargets());
        SmartDashboard.putString("ALIGN STATE", currentAlignState.toString());

        publisher.set(drivetrain.getState().Pose);
        publisher2.set(getPoseTesting());
     }

}