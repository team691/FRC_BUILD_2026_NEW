package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveToFuel extends Command {
    // add PID controllers instead probably for fine-tuned control
    // use pose estimator from tx, ty, ta, values
    
    private final DriveTrain m_drivetrain;
    private final PhotonCamera camera;

    public DriveToFuel(DriveTrain m_drivetrain, PhotonCamera camera) {
        this.m_drivetrain = m_drivetrain;
        this.camera = camera;

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
