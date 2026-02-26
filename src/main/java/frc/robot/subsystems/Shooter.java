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
//import frc.Buttons;

public class Shooter {
    TalonFX shooterMotor = null;

    public Shooter (int deviceID) {
        shooterMotor = new TalonFX(deviceID);
    }

    public void setShooterSpeed(int speed) {
        shooterMotor.set(speed);
    }

    public double getShooterSpeed() {
        return shooterMotor.get();
    }

    /*Final (Gear ratio = 1):
%Power = [1 / (200 * π * r)] *sqrt( [g * x^2] /[2 * cos^2(θ) * {x * tan(θ) - y}] )

For non-singular Gear Ratios:
%Power = [1 / (200 * π * r * G)] * sqrt( [g * x^2] /[2 * cos^2(θ) * {x * tan(θ) - y}] )
 */ 
}
