
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class Elevator extends SubsystemBase {
    private final TalonFX m_Elevator = new TalonFX(0);
    private final TalonFX m_followerElevator = new TalonFX(1);
    public MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

    ElevatorStates currentState = ElevatorStates.LOW;

    
    public static enum ElevatorStates {
        LOW,
        MEDIUM,
        HIGH
    }

    public Elevator() {
        elevatorConfigs();

        m_Elevator.setPosition(0);
        m_followerElevator.setPosition(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("ELEVATOR STATE", currentState.toString());
        switch (currentState){
            case LOW:
                toSetpoint(0);
                break;
            case MEDIUM:
                toSetpoint(1);
                break;
            case HIGH:
                toSetpoint(2);
                break;




        }
    }


    public void setState(ElevatorStates state){
        this.currentState = state;
    }

    public boolean atSetpoint(){
        return Util.inRange(
            m_Elevator.getPosition().getValueAsDouble() - mmVoltage.Position, 
            -1* Constants.Superstructure.kAtGoalTolerance, 
            Constants.Superstructure.kAtGoalTolerance);
    }
    public boolean atSetpoint2(){
        double tolerance = Constants.Superstructure.kAtGoalTolerance;
        double error = m_Elevator.getPosition().getValueAsDouble() - mmVoltage.Position;
        if( (error > -1 * tolerance) && (error < tolerance) ){
            return true;
        }else
        return false;
    }

    public void toSetpoint(double setHeight){
        mmVoltage = mmVoltage.withPosition(setHeight);
        m_Elevator.setControl(mmVoltage);

    }


    public void elevatorConfigs(){
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();


        talonFXConfigs.withSlot0(
            new Slot0Configs()
            .withKP(Constants.Elevator.kP)
            .withKI(Constants.Elevator.kI)
            .withKD(Constants.Elevator.kD)
            .withKS(Constants.Elevator.kS)
            .withKV(Constants.Elevator.kV)
            .withKA(Constants.Elevator.kA)
            .withKG(Constants.Elevator.kG)
            .withGravityType(GravityTypeValue.Elevator_Static)
        );


        talonFXConfigs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Elevator.kElevatorGearRatio)
        );

        talonFXConfigs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(Constants.Elevator.kMotionMagicAcceleration)
                .withMotionMagicCruiseVelocity(Constants.Elevator.kMotionMagicCruiseVelocity)
        );

        talonFXConfigs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        );

        m_Elevator.getConfigurator().apply(talonFXConfigs);
        m_followerElevator.getConfigurator().apply(talonFXConfigs);

        m_followerElevator.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
        );

        m_followerElevator.setControl(new StrictFollower(m_Elevator.getDeviceID()));


    }

        



}