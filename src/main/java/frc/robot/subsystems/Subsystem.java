
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
    private final TalonFX elevatormotor1 = new TalonFX(0);
    private final TalonFX elevatormotor2 = new TalonFX(1);

    private final MotionMagicVoltage motionmagic = new MotionMagicVoltage(0.0);

    //PIDSVAG

    public void motorconfigs(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.withSlot0(
            new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0)
            .withGravityType(GravityTypeValue.Elevator_Static)

        );
        
    }

    public Subsystem() {
    }

    private void gotoheight(int num)
    {
        motionmagic = motionmagic.withPosition(num);
        elevatormotor1.setControl(motionmagic);
        elevatormotor2.setControl(motionmagic);
        
    }

    private SubsystemStates state = SubsystemStates.LOW;


    public static enum SubsystemStates {
        LOW,
        MEDIUM,
        HIGH
    }

    @Override
    public void periodic() {
        switch (state)
        {
            case LOW:
                gotoheight(0);

                break;
            case MEDIUM:
                gotoheight(1);
                break;
            case HIGH:
                gotoheight(2);
                break;

        }

    }


    //
    public SubsystemStates settheState(SubsystemStates setstate)
    {
        this.state=setstate;
        return state;
    }


    public SubsystemStates gettheState()
    {
        return state;
    }



}