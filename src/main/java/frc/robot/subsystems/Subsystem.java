package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Util;
import frc.util.Alert;

public class Subsystem extends SubsystemBase {
    /* 
     * 1. Congrats for navigating here. These are two motors that you guys learned how to make. Now, 
     * we're going to learn in depth about configurating it.
     * (Reminder all types of comments using // is for code, all ones using /* are for instructions)
    */
    TalonFX motor = new TalonFX(0);
    TalonFX motorSlave = new TalonFX(1);



    public Subsystem() {
        /* 
         * 2. Now we've defined the motor, navigate to the configMotors method. Go to step three
         */
        configMotors();
        motor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Speed", motor.get());
    }


    private void configMotors() {
        /*
         * 3. Let's press control + click on the TalonFXConfiguration class below. That will take you to a page
         * where it explains all the different configs. Use this to fill in the remaining commented code for 
         * the different configs there are. Some of these you will have already encountered, some you may have not.
         * 
         * Use this opportunity to read up in the page about the different things that each config does. These
         * will be essential to running our motors.
         * 
         * After this, look at the slides for tuning and take notes on those.
         */
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.withFeedback(
            new FeedbackConfigs()
            .FeedbackSensorSource()

        );

        config.withSlot0(
            new Slot0Configs()
            .withKP()
            .withKI()


        );
        
        config.withMotionMagic(
            new MotionMagicConfigs()
            .MotionMagicCruiseVelocity()
        );

        config.withMotorOutput(
            new MotorOutputConfigs()
            .Inverted()
        );

        config.getConfigurator().apply(new ConfiguratorOutput());
        

        config.setNeutralMode(NeutralModeValue.Brake);

        config.setControl(NeutralModeValue.Brake request);
    }
}