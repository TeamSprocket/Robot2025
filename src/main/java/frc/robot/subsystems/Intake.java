// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Outtake.OuttakeStates;

public class Intake extends SubsystemBase {

  private final TalonFX intakemotor = new TalonFX(RobotMap.Intake.ROLL_INTAKE);
  private final PWM servo = new PWM(0);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private IntakeStates state = IntakeStates.NONE;

  private final SendableChooser<IntakeStates> stateChooser = new SendableChooser<>();

  public enum IntakeStates {
    NONE,
    STOWED,
    INTAKE,
    EJECT,
    CLIMB,
    INTAKE_REVERSE
  }

  /** Creates a new Intake. */
  public Intake() {

    stateChooser.setDefaultOption("NONE", IntakeStates.NONE);
    stateChooser.addOption("STOWED", IntakeStates.STOWED);
    stateChooser.addOption("INTAKE", IntakeStates.INTAKE);
    stateChooser.addOption("EJECT", IntakeStates.EJECT);
    SmartDashboard.putData("Intake State Chooser", stateChooser);

    TalonFXConfiguration IntakeConfig = new TalonFXConfiguration();

    IntakeConfig.withSlot0(
            new Slot0Configs()
                .withKS(Constants.Intake.kIntakeS) 
                .withKV(Constants.Intake.kIntakeV) 
                .withKG(Constants.Intake.kIntakeG)
                .withKA(Constants.Intake.kIntakeA) 
                .withKP(Constants.Intake.kIntakeP) 
                .withKI(Constants.Intake.kIntakeI) 
                .withKD(Constants.Intake.kIntakeD) 
    );

    IntakeConfig.withFeedback(
        new FeedbackConfigs()
          .withSensorToMechanismRatio(Constants.Intake.kIntakeGearRatio)
    );

    intakemotor.getConfigurator().apply(IntakeConfig);
    intakemotor.setNeutralMode(NeutralModeValue.Brake);

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putString("INTAKE STATE", state.toString());
    // setState(stateChooser.getSelected()); // TODO: remove later

    switch (state) {
      case NONE:
        intakemotor.setControl(velocityVoltage.withVelocity(0));
        break;

      case STOWED:
        intakemotor.setControl(velocityVoltage.withVelocity(Constants.Intake.kSpeedStowed));
        break;
          
      case INTAKE:
        intakemotor.setControl(velocityVoltage.withVelocity(Constants.Intake.kSpeedIntake));
        break;

      case EJECT:
        intakemotor.setControl(velocityVoltage.withVelocity(Constants.Intake.kSpeedEject));
        break;

      case CLIMB:
        intakemotor.setControl(velocityVoltage.withVelocity(0));
        servo.setPosition(1);
        break;
      // This method will be called once per scheduler run
    }
  }

  public void setState(IntakeStates state) {
    this.state = state;
  }

  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    intakemotor.setNeutralMode(neutralModeValue);
  }

  public IntakeStates getState() {
    return state;
  }

  public void revertIntake() {
    setState(IntakeStates.EJECT);
  }

  public double getIntakeSpeed() {
    return intakemotor.getVelocity().getValueAsDouble();
  }
}
