package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LimelightHelpers;

public class Vision implements Subsystem {
    public Pose2d globalPose;
    public LimelightHelpers.PoseEstimate mt2;
    public final SwerveDrivePoseEstimator poseEstimator;
    public final DriveTrain m_drivetrain = DriveTrain.getInstance();

    // initialize
    private static final Vision m_vision = new Vision();
    public static Vision getInstance() {return m_vision;}

    public Vision() {
        this.mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelight_three);
        this.poseEstimator = new SwerveDrivePoseEstimator(null, null, null, globalPose);
    }

    // sets robot orientation for limelight three
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(LimelightConstants.limelight_three, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public void runPosePeriodic() {
        poseEstimator.update(m_drivetrain.getGyroRotation(), m_drivetrain.getSwerveModulePositions());
    }

    public Pose2d globalPoseEstimator() {
        poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        return poseEstimator.getEstimatedPosition();
    }

    public final Pose2d estimateTargetPose(Pose2d robotPose, double txDeg, double tyDeg, double ta) {
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

    // TODO: test limelight localization code for global pose
    // TODO: climber autoalignment to apriltag (offset primarily)
    // TODO: reverse set gyro/reverse smth on robot?
}
