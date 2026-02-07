package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class estimatePose {
    public final Pose2d estimateTargetPose(Pose2d robotPose, double txDeg, double tyDeg, double ta) {
        double distance = 0.0; // figure out distance thing
        double headingOffsetRad = Math.toRadians(txDeg);

        double xRobot = distance * Math.cos(headingOffsetRad);
        double yRobot = distance * Math.sin(headingOffsetRad);

        Transform2d robotToTarget = 
            new Transform2d(new Translation2d(xRobot, yRobot), new Rotation2d(headingOffsetRad));

        return robotPose.transformBy(robotToTarget);
    }
}
