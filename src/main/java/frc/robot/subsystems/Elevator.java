//IMPORTS
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

public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
    private final TalonFX elevatorFollowerMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);

    private final Alert elevatorMotorDisconnected = new Alert("Elevator Motor Is Disconnected", Alert.AlertType.WARNING);

    private MotionMagicVoltage mmControl = new MotionMagicVoltage(0);

    //ELEVATOR STATES  
    public static enum ElevatorStates {
        NONE,
        STOWED,
        CORAL_2,
        CORAL_3,
        ALGAE_REMOVE_2,
        ALGAE_REMOVE_3
        // TESTING,
        // TESTING2,
        // TESTING3
    }

    private ElevatorStates state = ElevatorStates.NONE;
    
    private final SendableChooser<ElevatorStates> stateChooser = new SendableChooser<>();

    public Elevator() {
        configMotors();

        stateChooser.setDefaultOption("NONE", ElevatorStates.NONE);
        stateChooser.addOption("STOWED", ElevatorStates.STOWED);
        stateChooser.addOption("CORAL_2", ElevatorStates.CORAL_2);
        stateChooser.addOption("CORAL_3", ElevatorStates.CORAL_3);
        stateChooser.addOption("ALGAE_REMOVE_2", ElevatorStates.ALGAE_REMOVE_2);
        stateChooser.addOption("ALGAE_REMOVE_3", ElevatorStates.ALGAE_REMOVE_3);
        // stateChooser.addOption("TESTING", ElevatorStates.TESTING);
        // stateChooser.addOption("TESTING2", ElevatorStates.TESTING2);
        // stateChooser.addOption("TESTING3", ElevatorStates.TESTING3);
        SmartDashboard.putData("Elevator State Chooser", stateChooser);

        elevatorMotor.setPosition(0);
        elevatorFollowerMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // setState(stateChooser.getSelected());
        SmartDashboard.putString("ELEVATOR STATE", state.toString());
        switch (state) {
            case NONE:
                elevatorMotor.set(0);
                break;

            case STOWED:
                moveToHeight(Constants.Elevator.kHeightStowed);
                break;

            case CORAL_2:
                moveToHeight(Constants.Elevator.kHeightCoral2);
                break;

            case CORAL_3:
                moveToHeight(Constants.Elevator.kHeightCoral3);
                break;

            case ALGAE_REMOVE_2:
                moveToHeight(Constants.Elevator.kHeightAlgaeRemove2);
                break;

            case ALGAE_REMOVE_3:
                moveToHeight(Constants.Elevator.kHeightAlgaeRemove3);
                break;
            // case TESTING:
            //     elevatorMotor.setControl(new VoltageOut(0.90));
            // case TESTING2:
            //     elevatorMotor.setControl(new VoltageOut(0.93));
            // case TESTING3:
            //     elevatorMotor.setControl(new VoltageOut(0.96));
        }

        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Elevator State", state.toString());
    }

    public void setState(ElevatorStates newState) {
        this.state = newState;
    }

    private void moveToHeight(double targetHeight) {
        mmControl = mmControl.withPosition(targetHeight);
        elevatorMotor.setControl(mmControl);
    }

    public boolean atSetpoint() {
        return Util.inRange(
            elevatorMotor.getPosition().getValueAsDouble() - mmControl.Position, 
            -1 * Constants.Superstructure.kAtGoalTolerance, 
            Constants.Superstructure.kAtGoalTolerance
        );
    }

    private void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Elevator.kElevatorGearRatio)
        );

        config.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Elevator.kMotionMagicCruiseVelocity)
                .withMotionMagicAcceleration(Constants.Elevator.kMotionMagicAcceleration)
        );

        config.withSlot0(
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

        // config.withFeedback(
        //     new FeedbackConfigs()
        //         .withSensorToMechanismRatio(Constants.Elevator.kElevatorGearRatio)
        // );

        config.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        );

        elevatorMotor.getConfigurator().apply(config);
        elevatorFollowerMotor.getConfigurator().apply(config);

        elevatorFollowerMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        );

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

        elevatorFollowerMotor.setControl(new StrictFollower(elevatorMotor.getDeviceID()));

    }
}