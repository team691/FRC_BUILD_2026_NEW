package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class Intake{
    TalonFX intakeTalonMotor = null;
    SparkMax intakeNeoMotor = null; 
    
    // one neo for spin another kraken for up and down


    public Intake (int deviceIDTalon, int deviceIDNeo) {
        intakeTalonMotor = new TalonFX(deviceIDTalon);
        intakeNeoMotor = new SparkMax(deviceIDNeo, MotorType.kBrushless);
    }

    public void moveIntakeUp() {
        intakeTalonMotor.set(0.5);
        Timer.delay(1);
        intakeTalonMotor.set(0);
    }
    
    public void moveIntakedown() {
        intakeTalonMotor.set(-0.5);
        Timer.delay(1);
        intakeTalonMotor.set(0);
    }

    public void roll(){
        intakeNeoMotor.set(0.5);
        Timer.delay(10);
        intakeNeoMotor.set(0);

    }
    
}

