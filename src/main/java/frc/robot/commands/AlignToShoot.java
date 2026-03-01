package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Vision;
import frc.robot.constants.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;

public class AlignToShoot extends Command {
    public final PathConstraints constraints;
    public final DriveTrain drivebase;

    private Command pathCommand;

    private static final AlignToShoot m_align = new AlignToShoot(DriveTrain.getInstance());
    public static AlignToShoot getInstance() {return m_align;}
    
    private final Vision m_vision = new Vision();

    public AlignToShoot(DriveTrain drivebase) {
        this.constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540),
            Units.degreesToRadians(720)
        );

        this.drivebase = drivebase;

        addRequirements(drivebase);
    }

    public Pose2d findClosestPose2d() {
        // idea is to have a couple of in built poses, find the closest pose to current robot position for shooting and return it
        Pose2d[] posesArray = {FieldPoseConstants.blueLeftHubShooterAlign, FieldPoseConstants.blueCenterHubShooterAlign, FieldPoseConstants.blueRightHubShooterAlign, FieldPoseConstants.redLeftHubShooterAlign, FieldPoseConstants.redCenterHubShooterAlign, FieldPoseConstants.redRightHubShooterAlign};
        Pose2d currPose = m_vision.globalPoseEstimator(); //drivebase.getPose()

        double bestDist = Double.POSITIVE_INFINITY;
        int bestIndex = 0;
        

        for (int i = 0; i < posesArray.length; i++) {
            double dist = posesArray[i].getTranslation().getDistance(currPose.getTranslation());

            if (dist < bestDist) {
                bestDist = dist;
                bestIndex = i;
            }
        }

        System.out.println("BEST DISTANCE TO SHOOTING POSE" + bestDist);

        return posesArray[bestIndex];
    }

    @Override
    public void initialize() {
        Pose2d targetPose = findClosestPose2d();
        pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
        // pathCommand.schedule();
        CommandScheduler.getInstance().schedule(pathCommand);
    }

    @Override
    public void execute() {
    
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && !pathCommand.isScheduled();
    }
}
