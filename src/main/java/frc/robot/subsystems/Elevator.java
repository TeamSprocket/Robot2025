//IMPORTS
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Alert;

public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_LEFT);
    private final TalonFX elevatorFollowerMotor = new TalonFX(RobotMap.Elevator.ELEVATOR_RIGHT);

    private final Alert elevatorMotorDisconnected = new Alert("Elevator Motor Is Disconnected", Alert.AlertType.WARNING);

    private MotionMagicVoltage mmControl = new MotionMagicVoltage(0);

    //ELEVATOR STATES  
    public static enum SSStates {
        NONE,
        STOWED,
        INTAKE,
        HANDOFF,
        CORAL_1,
        CORAL_2,
        CORAL_3,
        ALGAE_REMOVE_2,
        ALGAE_REMOVE_3,
        SHALLOW_CLIMB,
        DEEP_CLIMB
    }

    private SSStates state = SSStates.NONE;
    private final SendableChooser<SSStates> stateChooser = new SendableChooser<>();

    public Elevator() {
        configMotors();

        stateChooser.setDefaultOption("NONE", SSStates.NONE);
        stateChooser.addOption("STOWED", SSStates.STOWED);
        stateChooser.addOption("INTAKE", SSStates.INTAKE);
        stateChooser.addOption("HANDOFF", SSStates.HANDOFF);
        stateChooser.addOption("CORAL_1", SSStates.CORAL_1);
        stateChooser.addOption("CORAL_2", SSStates.CORAL_2);
        stateChooser.addOption("CORAL_3", SSStates.CORAL_3);
        stateChooser.addOption("ALGAE_REMOVE_2", SSStates.ALGAE_REMOVE_2);
        stateChooser.addOption("ALGAE_REMOVE_3", SSStates.ALGAE_REMOVE_3);
        stateChooser.addOption("SHALLOW_CLIMB", SSStates.SHALLOW_CLIMB);
        stateChooser.addOption("DEEP_CLIMB", SSStates.DEEP_CLIMB);

        SmartDashboard.putData("Elevator State Chooser", stateChooser);

        elevatorFollowerMotor.setControl(new MotionMagicVoltage(0));
    }

    @Override
    public void periodic() {
        setState(stateChooser.getSelected());

        switch (state) {
            case NONE:
                elevatorMotor.set(0);
                break;

            case STOWED:
                moveToHeight(Constants.Elevator.kHeightStowed);
                break;

            case INTAKE:
                moveToHeight(Constants.Elevator.kHeightIntake);
                break;

            case HANDOFF:
                moveToHeight(Constants.Elevator.kHeightHandoff);
                break;

            case CORAL_1:
                moveToHeight(Constants.Elevator.kHeightCoral1);
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

            case SHALLOW_CLIMB:
                moveToHeight(Constants.Elevator.kHeightShallowClimb);
                break;

            case DEEP_CLIMB:
                moveToHeight(Constants.Elevator.kHeightDeepClimb);
                break;
        }

        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Elevator State", state.toString());
    }

    public void setState(SSStates newState) {
        state = newState;
    }

    private void moveToHeight(double targetHeight) {
        mmControl = mmControl.withPosition(targetHeight);
        elevatorMotor.setControl(mmControl);
    }

    private void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

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

        elevatorMotor.getConfigurator().apply(config);
        elevatorFollowerMotor.getConfigurator().apply(config);

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
    }
}
