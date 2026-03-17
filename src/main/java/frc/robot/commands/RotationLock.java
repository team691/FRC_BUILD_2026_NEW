package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class RotationLock extends Command {
    // TODO: can swap out for LL asw if no time

    private final PhotonCamera camera;
    private final DriveTrain driveTrain;
    private final PIDController rotController;

    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public RotationLock(PhotonCamera camera, DriveTrain driveTrain) {
        this.camera = camera;
        this.driveTrain = driveTrain;
        this.rotController = new PIDController(kP, kI, kD);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        rotController.reset();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
                var targetPose = result.getBestTarget();
                Rotation2d targetHeading = Rotation2d.fromDegrees(targetPose.getYaw());

                // [-180, 180] => [0, 360]
                // double currHeading = DriveTrain.getInstance().getHeading()+180;

                Rotation2d currHeading = driveTrain.getPose().getRotation();
                double rotError = targetHeading.minus(currHeading).getDegrees();
                double angularVelocity = rotController.calculate(currHeading.getDegrees(), rotError);

                ChassisSpeeds speeds = new ChassisSpeeds(0, 0, angularVelocity);
                // drive
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
                driveTrain.setModuleStates(moduleStates);
            }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, 0, false, false);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
