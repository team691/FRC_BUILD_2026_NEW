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


public class Climber {
    TalonFX ClimberTalonMotor = null;
    TalonFX ClimberTalonMotor2 = null;

    public Climber (int deviceIDTalon, int deviceIDTalon2) {
        ClimberTalonMotor = new TalonFX(deviceIDTalon);
        ClimberTalonMotor2 = new TalonFX(deviceIDTalon2);
    }

    public void ClimberMoveUp() {
        //ClimberTalonMotor
        //ClimberTalonMotor2
    }
}
