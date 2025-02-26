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
        fiducialID = getTargetTag();
        endpointL = new Pose2d();
        switch ((int)fiducialID) {
            case 6:
                endpointL = Constants.Vision.poseAlignRedLeft6;
                break;
            case 7:
                endpointL = Constants.Vision.poseAlignRedLeft7;
                break;
            case 8:
                endpointL = Constants.Vision.poseAlignRedLeft8;
                break;
            case 9:
                endpointL = Constants.Vision.poseAlignRedLeft9;
                break;
            case 10:
                endpointL = Constants.Vision.poseAlignRedLeft10;
                break;
            case 11:
                endpointL = Constants.Vision.poseAlignRedLeft11;
                break;
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
        fiducialID = getTargetTag();
        endpointR = new Pose2d();
        switch ((int)fiducialID) {
            case 6:
                endpointR = Constants.Vision.poseAlignRedRight6;
                break;
            case 7:
                endpointR = Constants.Vision.poseAlignRedRight7;
                break;
            case 8:
                endpointR = Constants.Vision.poseAlignRedRight8;
                break;
            case 9:
                endpointR = Constants.Vision.poseAlignRedRight9;
                break;
            case 10:
                endpointR = Constants.Vision.poseAlignRedRight10;
                break;
            case 11:
                endpointR = Constants.Vision.poseAlignRedRight11;
                break;
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
            3,4.2,Rotation2d.fromDegrees(0));
        return testPose;
    }

    private int getTargetTag() {
      int tag = (int)LimelightHelper.getFiducialID(name);
      boolean testRed = false;
        for (int element : redReefAprilTag) {
          if (element == tag) {
              testRed = true;
              break;
          }
      }

      boolean testBlue = false;
      for (int element : blueReefAprilTag) {
          if (element == tag) {
              testBlue = true;
              break;
          }
      }

      if ((allianceColor == Alliance.Red) && testRed) {
        return tag;
      } else if ((allianceColor == Alliance.Blue) && testBlue) {
        return tag;
      } else {
        return -1;
      }
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