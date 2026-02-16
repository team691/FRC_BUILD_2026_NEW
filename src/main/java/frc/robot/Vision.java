package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

public class Vision implements Subsystem {
    public Pose2d globalPose;
    public LimelightHelpers.PoseEstimate mt2;
    public final SwerveDrivePoseEstimator poseEstimator;

    public Vision() {
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelight_three);
        poseEstimator = new SwerveDrivePoseEstimator(null, null, null, globalPose);
    }

    // sets robot orientation for limelight three
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(LimelightConstants.limelight_three, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public void globalPoseEstimator() {

    }
}
