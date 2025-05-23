package frc.robot.subsystems;

import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;
import java.util.Arrays;
import java.util.Optional;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.AllianceStationID;
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

public class Vision extends SubsystemBase {
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Current Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();

    private PIDController pidRotationAlign = new PIDController(4.5, 0, 0); //3.5 0 0
    private PIDController pidXAlign = new PIDController(3.0, 0, 0); //2.5 0 0
    private PIDController pidYAlign = new PIDController(3.0, 0, 0); //2.5 0 0

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

    double maxSpeed = 3.0;

    // double moveForwardAlignDisplacement = 0.9;

    Pose2d lastPose = new Pose2d();
    double lastTimeStamp = 0.0;

    LimelightHelper.PoseEstimate estimate;

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

    @Override
    public void periodic() {
        testPose = getPoseTesting();

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

    public void setAlignState(AlignStates alignState) {
        currentAlignState = alignState;
    }

    public Pose2d getPoseTesting() {
        testPose = new Pose2d(
            12.684, 3.078,Rotation2d.fromDegrees(60));
        return testPose;
    }

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

    public Pose2d getClosestTagEstimate() {
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

    public Pose2d getTargetTagLeft() {
        //CHECK IF SAME FOR RED AND BLUE
        Pose2d targetTag = getClosestTag();
        Pose2d targetPose = new Pose2d(targetTag.getX() + Constants.Vision.xOffset*Math.cos(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getY() + Constants.Vision.xOffset*Math.sin(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getRotation());
        return targetPose;
    }

    public Pose2d getTargetTagRight() {
        Pose2d targetTag = getClosestTag();
        Pose2d targetPose = new Pose2d(targetTag.getX() - Constants.Vision.xOffset*Math.cos(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getY() - Constants.Vision.xOffset*Math.sin(targetTag.getRotation().getRadians()+Math.PI/2), targetTag.getRotation());
        return targetPose;
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

    public void updateAlignPose() {
        if (LimelightHelper.getTV(name)) {
            estimate = LimelightHelper.getBotPoseEstimate_wpiBlue(name);
            Pose2d tag = getClosestTagEstimate();
            if (Math.sqrt(Math.pow(tag.getX()-estimate.pose.getX(), 2) + Math.pow(tag.getY()-estimate.pose.getY(), 2)) < maxDistance) {
                drivetrain.resetPose(estimate.pose);
            }
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

    public Pose2d resetPose() {
        if (hasTargets()) {
            Pose2d pose = getPose2d();
            drivetrain.resetTeleopPose(pose);
            return pose;
        } else {
            return drivetrain.getAutoBuilderPose();
        }
    }

    public double[] getAlignOffsetsRight() {
        double veloX = pidXAlign.calculate(drivetrain.getState().Pose.getX(), getTargetTagRight().getX());
        double veloY = pidYAlign.calculate(drivetrain.getState().Pose.getY(), getTargetTagRight().getY());

        if (!Util.inRange(veloX, -maxSpeed, maxSpeed)) {
            veloX = (veloX / Math.abs(veloX)) * maxSpeed;
        }

        if (!Util.inRange(veloY, -maxSpeed, maxSpeed)) {
            veloY = (veloY / Math.abs(veloY)) * maxSpeed;
        }

        if (Util.inRange(veloX, -0.05, 0.05)) {
            veloX = 0.0;
        }

        if (Util.inRange(veloY, -0.05, 0.05)) {
            veloY = 0.0;
        }
        
        double[] values = {
          veloX, veloY
        };
        return values;
      }
    
      public double[] getAlignOffsetsLeft() {
        double speedX = pidXAlign.calculate(drivetrain.getState().Pose.getX(), getTargetTagLeft().getX());
        double speedY = pidYAlign.calculate(drivetrain.getState().Pose.getY(), getTargetTagLeft().getY());

        if (Util.inRange(speedX, -0.05, 0.05)) {
            speedX = 0.0;
        }

        if (Util.inRange(speedY, -0.05, 0.05)) {
            speedY = 0.0;
        }

        double[] values = {
          speedX, speedY
        };
        return values;
      }

    //   public double getMoveFowardAlignSpeed() {
        
    //   }
      
      public double getRotationalAlignSpeedRight() {
        pidRotationAlign.enableContinuousInput(0, 2*Math.PI);
        double currentRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double targetRotation = getTargetTagRight().getRotation().getRadians();
        
        double targetSpeed = pidRotationAlign.calculate(currentRotation, targetRotation);
        // System.out.println(targetSpeed);
        return targetSpeed;
      }
    
      public double getRotationalAlignSpeedLeft() {
        pidRotationAlign.enableContinuousInput(0, 2*Math.PI);
        double currentRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double targetRotation = getTargetTagLeft().getRotation().getRadians();
        
        double targetSpeed = pidRotationAlign.calculate(currentRotation, targetRotation);
        // System.out.println(currentRotation);
        // System.out.println(targetRotation);
        // System.out.println(targetSpeed);
        return targetSpeed;
      }
    
    private void debug() {
        SmartDashboard.putBoolean("Has Reef Target [VI]", hasReefTargets());
        SmartDashboard.putNumber("APRILTAG POSE", getPose2d().getX());
        SmartDashboard.putNumber("dist to left", distToAprilLeft);
        SmartDashboard.putNumber("dist to right", distToAprilRight);
        SmartDashboard.putNumber("times reset", counter);
        SmartDashboard.putBoolean("has targets", hasTargets());
        SmartDashboard.putString("ALIGN STATE", currentAlignState.toString());

        publisher.set(drivetrain.getState().Pose);
        publisher2.set(getTargetTagRight());
     }

}