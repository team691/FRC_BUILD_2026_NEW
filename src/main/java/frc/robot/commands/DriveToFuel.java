package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.constants.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveToFuel extends Command {
    // add PID controllers instead probably for fine-tuned control
    // use pose estimator from tx, ty, ta, values

    private final DriveTrain m_drivetrain;
    private static final PhotonCamera camera = new PhotonCamera(PhotonVisionConstants.cameraName);

    private static final DriveToFuel m_drivetofuel = new DriveToFuel(DriveTrain.getInstance(), camera);
    public static DriveToFuel getInstance() {return m_drivetofuel; }
    private final Vision m_vision;

    public DriveToFuel(DriveTrain m_drivetrain, PhotonCamera camera) {
        CameraServer.startAutomaticCapture();

        m_vision = new Vision();
        this.m_drivetrain = m_drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            m_drivetrain.drive(0, 0, 0, false, false);
            return;
        }
        
        var target = result.getBestTarget();        
        double yaw = target.getYaw();
        
        double turnCmd = PhotonVisionConstants.kPTurn * yaw;
        double forwardCmd = PhotonVisionConstants.forwardSpeed;

        m_drivetrain.drive(forwardCmd, forwardCmd, turnCmd, true, true);
    }

    public void getPose() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        double pitch = target.getPitch();
        double yaw = target.getYaw();
        double area = target.getArea();

        Pose2d estimatedPose = m_vision.estimateLimelightTargetPose(m_vision.globalPoseEstimator(), pitch, yaw, area);

        System.out.println("estimated target pose for ballsyness: " + estimatedPose);
    }

    public void distToFuel() {
        double dist = -1  ;
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            System.out.println("No Fuel detected");
        } else {
            
            //Using the finished robot, we need to find the actual calibration constant
            //see programming master doc for more details
            
            /*double k = knownDistance * Math.sqrt(target.getArea());

            double distance = k / Math.sqrt(target.getArea());
            double xPos = distance * Math.cos(Math.toRadians(target.getYaw()));
            double yPos = distance * Math.sin(Math.toRadians(target.getYaw()));
            */
            System.out.println(dist);
        }
    }

    @Override
    public boolean isFinished() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return false;
        }

        var target = result.getBestTarget();
        return target.getArea() > PhotonVisionConstants.areaThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0.0, 0.0, 0.0, false, false);
    }
   }
