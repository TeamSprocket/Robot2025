package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Util;
import frc.util.Alert;

public class Subsystem extends SubsystemBase {
    /* 
     * 1. Congrats for navigating here. Let's start here and make a motor, shall we?
     * Using what you know about motors, make and define a motor called SubsystemMotor. Make it private and final
     * Move onto part two when you're done
     * (Reminder all types of comments using // is for code, all ones using /* are for instructions)
    */


    public Subsystem() {
        /* 
         * 2. Now we've defined the motor, navigate to the configMotors method. Go to step three
         * Also fill in your motor name below in the setPosition line of code
         */
        configMotors();
        //.setPosition(0);
    }

    @Override
    public void periodic() {
        /*
         * 6. make a SmartDashboard function to send the value up to be displayed in Elastic
         * After, move to the RobotContainer.java file
         */

    }

    /*
     * 4. Make a method below to run the motor at a speed of 0.5, calling the method runMotor with a return of void
     */




    /*
     * 5. Make a method below to get the motor's position value, and return it for analysis and logging
     */


    private void configMotors() {
        /*
         * 3. You have some commented out code below. Figure out how to make each part of these configs work. 
         * I've provided you with the config variable already. 
         * HINT: Control + m1 on the class definitions
         */
        TalonFXConfiguration config = new TalonFXConfiguration();

        //config.withFeedback();

        //config.withMotorOutput();

        //.getConfigurator().apply(config);

        //.setNeutralMode(NeutralModeValue.Brake);
    }
}