package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.*;

public class Pivot extends SubsystemBase {
    MotionMagicVoltage mm = new MotionMagicVoltage(0);
    private final TalonFX motor = new TalonFX(RobotMap.AlgaePivot.ALGAE_PIVOT);
    PivotStates currentState = PivotStates.NONE;

    private final SendableChooser<PivotStates> stateChooser = new SendableChooser<>();
    
    public enum PivotStates {
      NONE,
      STOWED,
      ALGAE_REMOVE,
      INTAKE,
      L4
    }

  
    public Pivot(){

      stateChooser.setDefaultOption("NONE", PivotStates.NONE);
      stateChooser.addOption("STOWED", PivotStates.STOWED);
      stateChooser.addOption("ALGAE_REMOVE_2", PivotStates.ALGAE_REMOVE);
      SmartDashboard.putData("Pivot State Chooser", stateChooser);

      TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
      talonFXConfigs.withSlot0(
        new Slot0Configs()
          .withKS(Constants.Pivot.kS)//0.24
          .withKV(Constants.Pivot.kV)//7
          .withKA(Constants.Pivot.kA)//0.001
          .withKG(Constants.Pivot.kG) //-0.42
          .withKP(Constants.Pivot.kP)
          .withKI(Constants.Pivot.kI)
          .withKD(Constants.Pivot.kD)
          .withGravityType(GravityTypeValue.Arm_Cosine)
      );

      talonFXConfigs.withFeedback(
        new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.Pivot.kPivotGearRatio)
      );

      talonFXConfigs.withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(Constants.Pivot.kMotionMagicAcceleration)
        .withMotionMagicCruiseVelocity(Constants.Pivot.kMotionMagicCruiseVelocity)
      );

      motor.getConfigurator().apply(talonFXConfigs, 0.050);
      mm.Slot = 0;

      motor.setPosition(-0.2744);
    }

    @Override
    public void periodic() {
      SmartDashboard.putString("State [AP]", currentState.toString());
      // setState(stateChooser.getSelected());

      switch(currentState){
        case NONE:
        motor.set(0);
        break;

        case STOWED:
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setControl(mm.withPosition(Constants.Pivot.kAngleStowed));
        break;

        case ALGAE_REMOVE:
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setControl(mm.withPosition(Constants.Pivot.kAngleAlgaeRemove));
        break;

        case INTAKE:
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setControl(mm.withPosition(Constants.Pivot.kAngleIntake));
        break;

        case L4:
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setControl(mm.withPosition(Constants.Pivot.kAngleL4));
        break;

      }
    }

  public void setState(PivotStates state) {
    this.currentState = state;
  }

  public Command setStateCmd() {
    return new InstantCommand(() -> setState(PivotStates.STOWED));
  }

  public boolean atSetpoint() {
    return Util.inRange(
        motor.getPosition().getValueAsDouble() - mm.Position, 
        -1 * Constants.Superstructure.kAtGoalTolerance, 
        Constants.Superstructure.kAtGoalTolerance
    );
  }

  public double getShooterPivotAngle() {
    double deg = Conversions.falconToDegrees(motor.getRotorPosition().getValueAsDouble(), Constants.Pivot.kPivotGearRatio);

    deg %= 360;
    if (deg > 180) {
      deg -= (360); 
    }
    return deg;
  }
}