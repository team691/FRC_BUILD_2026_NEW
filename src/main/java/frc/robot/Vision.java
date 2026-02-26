package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LimelightHelpers;

public class Vision implements Subsystem {
    public LimelightHelpers.PoseEstimate mt2;
    // public Pose2d mt1;
    public final SwerveDrivePoseEstimator poseEstimator;
    public final DriveTrain m_drivetrain = DriveTrain.getInstance();

    // initialize
    private static final Vision m_vision = new Vision();
    public static Vision getInstance() {return m_vision;}

    public Vision() {
        this.mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.limelight_three);
        this.poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drivetrain.getGyroRotation(), m_drivetrain.getSwerveModulePositions(), m_drivetrain.getPose());
    }

    public void configurePhotonVision() {
        
    }

    // sets robot orientation for limelight three
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(LimelightConstants.limelight_three, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public void runPosePeriodic() {
        poseEstimator.update(m_drivetrain.getGyroRotation(), m_drivetrain.getSwerveModulePositions());
    }
int testvalue = 0;
    public Pose2d globalPoseEstimator() {
        // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        // // poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        // poseEstimator.addVisionMeasurement(mt1, testvalue);
        // testvalue += 1;
        // System.out.println("mt1 pose stuff: " + mt1);
        // // System.out.println("mt1.pose: " + mt1.pose);
        // // System.out.println("timestamp: " + mt1.timestampSeconds);
        // System.out.println("testvalue: " + testvalue);
        // return poseEstimator.getEstimatedPosition();
        boolean doRejectUpdate = false;
        if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
            if (mt2.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt2.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        return poseEstimator.getEstimatedPosition();
    }

    public final Pose2d estimateLimelightTargetPose(Pose2d robotPose, double txDeg, double tyDeg, double ta) {
        double distance = (LimelightConstants.cameraHeight - LimelightConstants.targetHeight)/Math.tan(LimelightConstants.mountAngle + tyDeg); // figure out distance thing
        // (target height - camera height)/tan(limelight moutning angle + ty)
        double headingOffsetRad = Math.toRadians(txDeg);

        double xRobot = distance * Math.cos(headingOffsetRad);
        double yRobot = distance * Math.sin(headingOffsetRad);

        Transform2d robotToTarget = 
            new Transform2d(new Translation2d(xRobot, yRobot), new Rotation2d(headingOffsetRad));

        return robotPose.transformBy(robotToTarget);
    }


    // TODO: add method for driving directly to detected blob pose on photon vision
    public Command colorBlobDrive() {

        return null;
    }

    // TODO: test limelight localization code for global pose (can we get our current pose with limelight?)
    // TODO: climber autoalignment to apriltag (offset primarily)
    // TODO: reverse set gyro/reverse smth on robot?
}
