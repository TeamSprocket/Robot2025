package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.LimelightHelper;
import frc.util.Util;

public class Vision extends SubsystemBase {
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Endpoint", Pose2d.struct).publish();

    private int[] blueReefAprilTag = {17, 18, 19, 20, 21, 22};
    private int[] redReefAprilTag = {6, 7, 8, 9, 10, 11};

    CommandSwerveDrivetrain drivetrain;

    String name = "limelight-front";

    Pose2d lastPose = new Pose2d();
    double lastTimeStamp = 0.0;

    LimelightHelper.PoseEstimate estimate;

    Pose2d endpointL = new Pose2d();
    Pose2d endpointR = new Pose2d();

    Pose2d robotPose = new Pose2d();

    Command pathL;
    Command pathR;

    double distToAprilLeft = 0.0;
    double distToAprilRight = 0.0;
    boolean updateFirst = true;

    double fiducialID;

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
    }

    @Override
    public void periodic() {
        publisher.set(drivetrain.getAutoBuilderPose());
        debug();

        updateAlignPose();

        if (hasTargets() && updateFirst) {
            resetPose();
            updateFirst = false;
        } else if (!hasTargets() && !updateFirst) {
            updateFirst = true;
        }
    }

    public Command getAlignPath(boolean pathDirection) {
        addVisionPose();
        fiducialID = LimelightHelper.getFiducialID(name);
        endpointL = new Pose2d(); // used for both sides

        if (pathDirection) { 
                switch ((int)fiducialID) {
                    case 17:
                        endpointL = Constants.Vision.poseAlignBlueRight17;
                        break;
                    case 18:
                        endpointL = Constants.Vision.poseAlignBlueRight18;
                        break;
                    case 19:
                        endpointL = Constants.Vision.poseAlignBlueRight19;
                        break;
                    case 20:
                        endpointL = Constants.Vision.poseAlignBlueRight20;
                        break;
                    case 21:
                        endpointL = Constants.Vision.poseAlignBlueRight21;
                        break;
                    case 22:
                        endpointL = Constants.Vision.poseAlignBlueRight22;
                        break;
                    case -1:
                        endpointL = drivetrain.getAutoBuilderPose();
                        break;
                }
        } else {
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
        }

        return AutoBuilder.pathfindToPose(
        endpointL,
        new PathConstraints(2, 2, 3, 2), 0.0
        );

    }

    public Command getAlignPathLeft() {
        addVisionPose();
        fiducialID = LimelightHelper.getFiducialID(name);
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
    

        // pathL = AutoBuilder.pathfindToPose(
        //     endpointL,
        //     new PathConstraints(4, 3, 4, 2), 
        //     0.0
        // );

        // return pathL;

        return getPathfindToPose(endpointL);
    }

    public Command getAlignPathRight() {
        addVisionPose();
        fiducialID = LimelightHelper.getFiducialID(name);
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

        // pathR = AutoBuilder.pathfindToPose(
        //     endpointR,
        //     new PathConstraints(4, 3, 4, 2), 
        //     0.0
        // );

        // return pathR;

        return getPathfindToPose(endpointR);
    }

    public Command getPathfindToPose(Pose2d endpoint) {
        return AutoBuilder.pathfindToPose(
            endpoint,
            new PathConstraints(2, 2, 3, 2), 0.0
        );
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

    public void updateAlignPose() {
        if (LimelightHelper.getTV(name)) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            drivetrain.alignRobotPose = estimate.pose;
        } else {
            drivetrain.alignRobotPose = drivetrain.getAutoBuilderPose();
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

    public Pose2d addVisionPose() {
        if (hasTargets()) {
            Pose2d pose = getPose2d();
            drivetrain.addVisionMeasurement(pose, getTimeStamp());
            return pose;
        } else {
            return drivetrain.getAutoBuilderPose();
        }
    }

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
        SmartDashboard.putNumber("FUDICIAL ID", LimelightHelper.getFiducialID(name));
        SmartDashboard.putNumber("END XL",endpointL.getX());
        SmartDashboard.putNumber("END YL",endpointL.getY());
        SmartDashboard.putNumber("END XR",endpointR.getX());
        SmartDashboard.putNumber("END YR",endpointR.getY());
        SmartDashboard.putNumber("dist to left", distToAprilLeft);
        SmartDashboard.putNumber("dist to right", distToAprilRight);
     }

}
