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
import frc.robot.constants.Constants.*;

public class Shooter extends SubsystemBase {
    private static final Shooter m_shooter = new Shooter();
    public static Shooter getInstance() {return m_shooter;}
    public static final String cam_thing = "HD_USB_Camera";
    private final PhotonCamera camza = new PhotonCamera(cam_thing);
    Transform3d position = new Transform3d();
     HashMap<Integer, Float> AprilTagHashMap = new HashMap<>(
    // {11, 50},
    );

    TalonFX shooterMotor;

    public Shooter () {
        shooterMotor = new TalonFX(ShooterConstants.shooterTalonDeviceId);
        AprilTagHashMap.put(2, 50.0f);
        AprilTagHashMap.put(11, 50.0f);
        AprilTagHashMap.put(3, 53.0f);
        AprilTagHashMap.put(4, 53.0f);
    }

    // TODO: when we are not shooting/not our turn, set shooter speed to like 0.75 (constant) for moving balls to other side
    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public double getShooterSpeed(double distance, double angle_to_hub, Float constant) {
        // return shooterMotor.get();
        double speed = (2.05*distance+0.045*angle_to_hub + constant)/100;
        return speed;
    }
    

    public void teleopPeriodic() {
    // double speed = (2.05*dist+0.045*angle_to_hub + 41)/100;
    PhotonPipelineResult result = camza.getLatestResult();
    camza.setPipelineIndex(0);

    if (result.hasTargets()) {
  if (camza.getPipelineIndex() == 0) {
    
    var target = result.getBestTarget();
    int id = target.getFiducialId();
    Transform3d bestCam = target.getBestCameraToTarget();
    System.out.println("stuffx: " + bestCam.getX());
    System.out.println("stuffy: " + bestCam.getY());

    double distance = (Math.sqrt(Math.pow(bestCam.getX(), 2) + Math.pow(bestCam.getY(), 2))) * (3.28084) + 4.25;
    double angle_to_hub = target.getYaw();
    Float constant = AprilTagHashMap.get(id);
    double speed = getShooterSpeed(distance, angle_to_hub, constant);
    setShooterSpeed(speed);
    
  }}}
    

    /*Final (Gear ratio = 1):
%Power = [1 / (200 * π * r)] *sqrt( [g * x^2] /[2 * cos^2(θ) * {x * tan(θ) - y}] )

For non-singular Gear Ratios:
%Power = [1 / (200 * π * r * G)] * sqrt( [g * x^2] /[2 * cos^2(θ) * {x * tan(θ) - y}] )
 */ 
}
