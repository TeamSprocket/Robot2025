package frc.robot.subsystems;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
// import frc.robot.Constants.Vision;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.util.LimelightHelper;
import frc.util.ShuffleboardIO;
import frc.util.Util;


/**
 * The Vision subsystem gathers and calculates information that deals with the position of the robot and april tags
 * using LimeLight with odometry to find an estimation on the robot or tag pose2d
 * can also be used to return alignment values on the reef or used to check and return PID Offsets
 * 
 */
public class Vision extends SubsystemBase {
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Current Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();

    private PIDController pidRotationAlign = new PIDController(3.5, 0, 0); //4.5 0 0
    private PIDController pidXAlign = new PIDController(2.0, 0, 0); //3.0 0 0
    private PIDController pidYAlign = new PIDController(2.0, 0, 0); //3.0 0 0

    Timer timer = new Timer();

    Pose2d testPose = new Pose2d();

    public enum AlignStates{
        ALIGNING,
        NONE
    }

    AlignStates currentAlignState = AlignStates.NONE;

    Alliance allianceColor = Alliance.Blue;

    private int[] blueReefAprilTag = {17, 18, 19, 20, 21, 22};
    private int[] redReefAprilTag = {6, 7, 8, 9, 10, 11};

    CommandSwerveDrivetrain drivetrain;

    String name = "limelight-front";
    int counter = 0;
    int tagOutside = 1;

    double maxSpeed = 2.0;

    // double moveForwardAlignDisplacement = 0.9;

    // Pose2d lastPose = new Pose2d();
    double lastTimeStamp = 0.0;

    LimelightHelper.PoseEstimate visionEstimate;

    Command pathL;
    Command pathR;

    double distToAprilLeft = 0.0;
    double distToAprilRight = 0.0;
    boolean updateFirst = true;

    double maxDistance = 1.85;

    double fiducialID;

    
    

    public Vision(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
        timer.reset();
        timer.start();

        
        ShuffleboardIO.addSlider("Alignment X", 0, 7, 0);
        ShuffleboardIO.addSlider("Alignment Y", 0, 7, 0);
    }



    /**
     * this method periodically uploads the target x,y speed, and debug into smartDashboard as well as updating the alignment pose
     * 
     * @see updateAlignPose();
     * @see debug();
     */
    @Override
    public void periodic() {
        if (LimelightHelper.getTV(name)) {
            LimelightHelper.SetRobotOrientation(name, drivetrain.getPigeon2().getYaw().getValueAsDouble(), drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);
            visionEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
        }

        SmartDashboard.putNumber("Target Speed X", getAlignOffsetsRight()[0]);
        SmartDashboard.putNumber("Target Speed Y", getAlignOffsetsRight()[1]);

        if (timer.get() > 0.2 && currentAlignState == AlignStates.ALIGNING) {
            updateAlignPose();
            // System.out.println("UPDATING");
            timer.reset();
            timer.start();
        }
        
        debug();
    }


    /**
     * This method sets the aligning state of the robot
     * @param alignState ALIGNING or NONE
     */
    public void setAlignState(AlignStates alignState) {
        currentAlignState = alignState;
    }

    /**
     * this method gets the closest tag pose in relation to the current robot pose
     * 
     * @return targetPose - pose2d
     */
    public Pose2d getClosestTag() {
        int tag = -1;
        double minDistance = Integer.MAX_VALUE;
        Pose2d targetPose = new Pose2d();
            for (int i = 6; i <= 11; i++) {
            Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
            if (i == 6) target = Constants.Vision.Red6;
            else if (i == 7) target = Constants.Vision.Red7;
            else if (i == 8) target = Constants.Vision.Red8;
            else if (i == 9) target = Constants.Vision.Red9;
            else if (i == 10) target = Constants.Vision.Red10;
            else if (i == 11) target = Constants.Vision.Red11;

            double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
            if (distance < minDistance) {
                minDistance = distance;
                tag = i;
                targetPose = target;
            }
        }

        for (int i = 17; i <= 22; i++) {
            Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
            if (i == 17) target = Constants.Vision.Blue17;
            else if (i == 18) target = Constants.Vision.Blue18;
            else if (i == 19) target = Constants.Vision.Blue19;
            else if (i == 20) target = Constants.Vision.Blue20;
            else if (i == 21) target = Constants.Vision.Blue21;
            else if (i == 22) target = Constants.Vision.Blue22;

            double distance = Util.distance(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), target.getX(), target.getY());
            if (distance < minDistance) {
                minDistance = distance;
                tag = i;
                targetPose = target;
            }
        }
        return targetPose;
    }

    /**
     * This method uses vision to estimate the bot pose then calculates the closest tag pose with that estimate
     * 
     * @return targetpose - pose2d
     */
    public Pose2d getClosestTagEstimate() {
    //TEST TO SEE IF THIS IS REDUNDANT OR NOT
        int tag = -1;
        double minDistance = Integer.MAX_VALUE;
        Pose2d targetPose = new Pose2d();
        LimelightHelper.PoseEstimate estimate2 = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            for (int i = 6; i <= 11; i++) {
            Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
            if (i == 6) target = Constants.Vision.Red6;
            else if (i == 7) target = Constants.Vision.Red7;
            else if (i == 8) target = Constants.Vision.Red8;
            else if (i == 9) target = Constants.Vision.Red9;
            else if (i == 10) target = Constants.Vision.Red10;
            else if (i == 11) target = Constants.Vision.Red11;

            double distance = Util.distance(estimate2.pose.getX(), estimate2.pose.getY(), target.getX(), target.getY());
            if (distance < minDistance) {
                minDistance = distance;
                tag = i;
                targetPose = target;
            }
        }

        for (int i = 17; i <= 22; i++) {
            Pose2d target = new Pose2d(0, 0, new Rotation2d(0));
            if (i == 17) target = Constants.Vision.Blue17;
            else if (i == 18) target = Constants.Vision.Blue18;
            else if (i == 19) target = Constants.Vision.Blue19;
            else if (i == 20) target = Constants.Vision.Blue20;
            else if (i == 21) target = Constants.Vision.Blue21;
            else if (i == 22) target = Constants.Vision.Blue22;

            double distance = Util.distance(estimate2.pose.getX(), estimate2.pose.getY(), target.getX(), target.getY());
            if (distance < minDistance) {
                minDistance = distance;
                tag = i;
                targetPose = target;
            }
        }
        return targetPose;
    }
    


    /**
     * Finds the pose2d left of the closest tag
     * 
     * @return targetPose - pose2d
     * @see getClosestTag();
     */
    public Pose2d getTargetTagLeft() {
        //CHECK IF SAME FOR RED AND BLUE
        Pose2d targetTag = getClosestTag();
        Pose2d targetPose = new Pose2d(targetTag.getX() + Constants.Vision.xOffset*Math.cos(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getY() + Constants.Vision.xOffset*Math.sin(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getRotation());
        return targetPose;
    }

      /**
     * Finds the pose2d right of the closest tag
     * 
     * @return targetPose - pose2d
     * @see getClosestTag();
     */
    public Pose2d getTargetTagRight() {
        Pose2d targetTag = getClosestTag();
        Pose2d targetPose = new Pose2d(targetTag.getX() - Constants.Vision.xOffset*Math.cos(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getY() - Constants.Vision.xOffset*Math.sin(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getRotation());
        return targetPose;
    }

    /**
     * @return {xCoord, yCoord, timestamp}
     */
    //CHECK TO SEE IF THIS CODE IS USEFUL
    // public Translation2d getTranslation2d() {
    //     LimelightHelper.PoseEstimate estimate;
    //     if (LimelightHelper.getTV(name)) {
    //         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //             estimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    //         }
    //         else {
    //             estimate = LimelightHelper.getBotPoseEstimate_wpiRed_MegaTag2(name);
    //         }
    //         return new Translation2d(estimate.pose.getX(), estimate.pose.getY());
    //     } else {
    //         return new Translation2d(0.0, 0.0);
    //     }
    // }



    /**
     * this method gets the current pose2d of the bot using LL estimate
     * 
     * @return lastPose - pose2d of the bot using LL
     */
    // public Pose2d getPose2d() {
    //     if (LimelightHelper.getTV(name)) {
    //         LimelightHelper.SetRobotOrientation(name, drivetrain.getPigeon2().getYaw().getValueAsDouble(), drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);
    //         estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
    //         lastPose = estimate.pose;
    //         return estimate.pose;
    //     } else {
    //         return lastPose;
    //     }
    // }


    /**
     * this method, if there is a target tag then returns the horizontal offset and if not returns 0
     * 
     * @return double - horizontal offset
     */
    // public double getTX() {
    // //THIS METHOD IS NOT REFRENCED ANYWHERE, MIGHT BE DEBUG
    //     if (hasTargets()) {
    //         return LimelightHelper.getTX(name);
    //     } else {
    //         return 0.0;
    //     }
    // }



    /**
     * this method updates the pose which the robot wants to align to using a kalman filter with vision and odometry inputs
     * 
     * @see getClosestTag();
     */
    public void updateAlignPose() {
        if (LimelightHelper.getTV(name)) {
            Pose2d tag = getClosestTagEstimate();
            if (Math.sqrt(Math.pow(tag.getX()-visionEstimate.pose.getX(), 2) + Math.pow(tag.getY()-visionEstimate.pose.getY(), 2)) < maxDistance) {
                // drivetrain.resetPose(estimate.pose);
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.02,0.02,0.01));
                drivetrain.addVisionMeasurement(visionEstimate.pose, visionEstimate.timestampSeconds);
            }
        }
    }


    /**
     * this method resets the alignment pose for the robot
     * 
     * @see getClosestTag();
     */
    public void resetAlignPose() {
        if (LimelightHelper.getTV(name)) {
            // var LLMeasurment = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            Pose2d tag = getClosestTag(); //getClosestTagEstimate()
            if (Math.sqrt(Math.pow(tag.getX()-visionEstimate.pose.getX(), 2) + Math.pow(tag.getY() - visionEstimate.pose.getY(), 2)) < maxDistance) {
                drivetrain.resetPose(visionEstimate.pose);
                // drivetrain.addVisionMeasurement(LLMeasurment.pose, LLMeasurment.timestampSeconds);
            }
        }
    }

    /**
     * gets the last timestamp of the robot
     * 
     * @return lastTimeStamp - double
     */
    // public double getTimeStamp() {
    //     //NOT REFERENCED ANYWHERE (MIGHT BE DEBUG)
    //     if (LimelightHelper.getTV(name)) {
    //         estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
    //         lastTimeStamp = estimate.timestampSeconds;
    //         return estimate.timestampSeconds;
    //     } else {
    //         return lastTimeStamp;
    //     }
    // }


    /**
     * returns true if the LL is looking at a target
     * 
     * @return boolean - true or false
     */
    public boolean hasTargets() {
        return LimelightHelper.getTV(name);
    }
    

    /**
     * returns true if the LL is looking at a reef tag
     * 
     * @return boolean - true or false
     */
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


    /**
     * resets the bot pose using LL if present and if not then uses autobuilderpose
     * 
     * @return pose - pose2d
     * @see getPose2d()
     */
    // public Pose2d resetPose() {
    //     if (hasTargets()) {
    //         Pose2d pose = getPose2d();
    //         drivetrain.resetTeleopPose(pose);
    //         return pose;
    //     } else {
    //         return drivetrain.getAutoBuilderPose();
    //     }
    // }


    /**
     * this method caps the max speed of PID output while preserving it's direction and includes a deadband for when there is minimal movement for the right side of the tag
     * 
     * @return {veloX, veloY} - array new PID outputs
     */
    public double[] getAlignOffsetsRight() {
        double veloX = pidXAlign.calculate(drivetrain.getState().Pose.getX(), getTargetTagRight().getX());
        double veloY = pidYAlign.calculate(drivetrain.getState().Pose.getY(), getTargetTagRight().getY());

        if (!Util.inRange(veloX, -maxSpeed, maxSpeed)) {
            veloX = (veloX / Math.abs(veloX)) * maxSpeed;
        }

        if (!Util.inRange(veloY, -maxSpeed, maxSpeed)) {
            veloY = (veloY / Math.abs(veloY)) * maxSpeed;
        }

        if (Util.inRange(veloX, -Constants.Vision.apriltagMinSpeed, Constants.Vision.apriltagMinSpeed)) {
            veloX = 0.0;
        }

        if (Util.inRange(veloY, -Constants.Vision.apriltagMinSpeed, Constants.Vision.apriltagMinSpeed)) {
            veloY = 0.0;
        }
        
        double[] values = {
          veloX, veloY
        };
        return values;
      }


      /**
     * this method caps the max speed of PID output while preserving it's direction and includes a deadband for when there is minimal movement for the left side of the tag
     * 
     * @return {veloX, veloY} - array of new PID outputs
     * 
     */
      public double[] getAlignOffsetsLeft() {
        double veloX = pidXAlign.calculate(drivetrain.getState().Pose.getX(), getTargetTagLeft().getX());
        double veloY = pidYAlign.calculate(drivetrain.getState().Pose.getY(), getTargetTagLeft().getY());

        if (!Util.inRange(veloX, -maxSpeed, maxSpeed)) {
            veloX = (veloX / Math.abs(veloX)) * maxSpeed;
        }

        if (!Util.inRange(veloY, -maxSpeed, maxSpeed)) {
            veloY = (veloY / Math.abs(veloY)) * maxSpeed;
        }

        if (Util.inRange(veloX, -Constants.Vision.apriltagMinSpeed, Constants.Vision.apriltagMinSpeed)) {
            veloX = 0.0;
        }

        if (Util.inRange(veloY, -Constants.Vision.apriltagMinSpeed, Constants.Vision.apriltagMinSpeed)) {
            veloY = 0.0;
        }

        double[] values = {
            veloX, veloY
        };
        return values;
      }
      

      /**
       * this method gets the rotational speed to align to the right of the tag
       * 
       * @return targetSpeed - the PID output for rotational speed
       */
      public double getRotationalAlignSpeedRight() {
        pidRotationAlign.enableContinuousInput(0, 2*Math.PI);
        double currentRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double targetRotation = getTargetTagRight().getRotation().getRadians();

        double targetSpeed = pidRotationAlign.calculate(currentRotation, targetRotation);
        return targetSpeed;
      }


      /**
       * this method gets the rotational speed to align to the left of the tag
       * 
       * @return targetSpeed - the PID output for rotational speed
       */
      public double getRotationalAlignSpeedLeft() {
        pidRotationAlign.enableContinuousInput(0, 2*Math.PI);
        double currentRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double targetRotation = getTargetTagLeft().getRotation().getRadians();
        
        double targetSpeed = pidRotationAlign.calculate(currentRotation, targetRotation);
        return targetSpeed;
      }
    
    /**
     * this method puts different values into smartDashboard for testing/debugging purposes
     */
    private void debug() {
        SmartDashboard.putBoolean("Has Reef Target [VI]", hasReefTargets());
        SmartDashboard.putNumber("Vision POSE X", visionEstimate.pose.getX());
        SmartDashboard.putNumber("Vision POSE Y", visionEstimate.pose.getY());
        SmartDashboard.putNumber("dist to left", distToAprilLeft);
        SmartDashboard.putNumber("dist to right", distToAprilRight);
        SmartDashboard.putNumber("times reset", counter);
        SmartDashboard.putBoolean("has targets", hasTargets());
        SmartDashboard.putString("ALIGN STATE", currentAlignState.toString());

        publisher.set(drivetrain.getState().Pose);
        publisher2.set(getTargetTagRight());
     }

}