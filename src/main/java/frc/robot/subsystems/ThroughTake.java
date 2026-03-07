package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.Buttons;
import dev.doglog.*;
import frc.robot.constants.Constants.*;

public class ThroughTake extends SubsystemBase {
  private SparkMax throughTakeMotor = new SparkMax(12, MotorType.kBrushless);

  public ThroughTake() {
    
  }

  public void runThroughTake() {
    throughTakeMotor.set(0.5);
    DogLog.log("ThroughTake", "ThroughTake running");
  }

  public void stopThroughTake() {
    throughTakeMotor.set(0);
    DogLog.log("ThroughTake", "ThroughTake stopped");
  }


}
