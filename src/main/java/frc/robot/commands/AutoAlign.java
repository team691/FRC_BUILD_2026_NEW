package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class AutoAlign extends Command {
    private final DriveTrain drivebase;
    private final HolonomicDriveController driveController;
    private final ProfiledPIDController thetaController;

    private final Timer stopTimer = new Timer();
    private final Timer tagLostTimer = new Timer();
    public static final String limelightStream = "";

    private static final AutoAlign m_align = new AutoAlign(DriveTrain.getInstance(), limelightStream);
    public static AutoAlign getInstance() {return m_align;}

    // private double tagID = -1;

    public AutoAlign(DriveTrain drivebase, String limelightStream) {
        this.drivebase = drivebase;

        thetaController = new ProfiledPIDController(
            Constants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = new HolonomicDriveController(
            new edu.wpi.first.math.controller.PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0.0),
            new edu.wpi.first.math.controller.PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0.0),
            thetaController
        );

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        stopTimer.reset();
        stopTimer.start();
        tagLostTimer.reset();
        tagLostTimer.start();
        // tagID = LimelightHelpers.getFiducialID("limelight");
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightStream)) {
            // No tag seen
            drivebase.drive(0, 0, 0, false, false);
            return;
        }

        tagLostTimer.reset();  // reset lost tag timer because we see a tag

        // Use robot pose relative to target — more stable
        PoseEstimate currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightStream);

        // Target pose is fixed relative to tag
        Pose2d targetPose = new Pose2d(
            Constants.X_SETPOINT_REEF_ALIGNMENT,
            Constants.Y_SETPOINT_REEF_ALIGNMENT, //  : -Constants.Y_SETPOINT_REEF_ALIGNMENT
            Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT)
        );

        Trajectory.State dummyState = new Trajectory.State(0, 0, 0, targetPose, 0);

        ChassisSpeeds speeds = driveController.calculate(currentPose.pose, dummyState, targetPose.getRotation());

        drivebase.drive(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false,
            false
        );

        SmartDashboard.putNumber("AlignTimer", stopTimer.get());
        
        boolean aligned = driveController.atReference();

        // Reset stop timer if not aligned
        if (!aligned) {
            stopTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return tagLostTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
            || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
