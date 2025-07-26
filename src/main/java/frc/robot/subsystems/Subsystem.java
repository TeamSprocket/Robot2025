
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Util;
import frc.util.Alert;

public class Subsystem extends SubsystemBase {
    private final TalonFX elevatorMotor = new TalonFX(0);
    private final TalonFX followerMotor = new TalonFX(0);
    private final SubsystemStates state = SubsystemStates.NONE;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(null);


    public static enum SubsystemStates {
        NONE,
        LOW, 
        MEDIUM,
        HIGH
    }

    public Subsystem() {

    }
private void configMotors(){
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    Slot0Configs slot0Configs = new Slot0Configs();

    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicAcceleration = 0;

    slot0Configs.withKS(0);
    slot0Configs.withKV(0);
    slot0Configs.withKA(0);
    slot0Configs.withKG(0);
    slot0Configs.withKP(0);
    slot0Configs.withKI(0);
    slot0Configs.withKD(0);


    elevatorMotor.getConfigurator().apply(talonFXConfigs);
    elevatorMotor.getConfigurator().apply(slot0Configs);
    elevatorMotor.getConfigurator().apply(motionMagicConfigs);
    followerMotor.getConfigurator().apply(talonFXConfigs);
    followerMotor.getConfigurator().apply(slot0Configs);
    followerMotor.getConfigurator().apply(motionMagicConfigs);

    



}
    @Override
    public void periodic() {
    switch (state) {
    case NONE:
        elevatorMotor.set(0);
        break;
    case LOW:
        moveToHeight(0);
        break;
    case MEDIUM:
        moveToHeight(1);
        break;
    case HIGH:
        moveToHeight(2);
    }

    private void moveToHeight(double height){
        
    }

    public void setState(SubsystemStates state){

    }
}