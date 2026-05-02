package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ThroughTake;

/*
Commands.sequence (
              Commands.runOnce(() -> Shooter.getInstance().setShooterRPM(500)),

              Commands.waitUntil(() ->
                  Math.abs(Shooter.getInstance().getShooterRPM() - 500) <= 50
              ),

              Commands.run(() -> ThroughTake.getInstance().runThroughTake(0.7))
                  .withTimeout(4),

              Commands.runOnce(() -> {
                  Shooter.getInstance().stopShooter();
                  ThroughTake.getInstance().stopThroughTake();
              })
    )
 */

public class ThroughTakeToShoot extends Command {
    int targetRPM = 500;
    int targetRPS = targetRPM/60;
    
    public ThroughTakeToShoot() {
        addRequirements(Shooter.getInstance(), ThroughTake.getInstance());
    }

    @Override
    public void initialize() {
  
        // Shooter.getInstance().stopShooter();
        // ThroughTake.getInstance().stopThroughTake();
    }

    @Override
    public void execute() {
              Shooter.getInstance().setShooterRPM(targetRPM);

        double tolerance = 1.0;

        // Commands.waitSeconds(1.0).andThen(() -> {
        //     ThroughTake.getInstance().runThroughTake(0.7);
        // });

        if (Math.abs(Shooter.getInstance().getShooterRPM() - targetRPM) <= tolerance) {
            ThroughTake.getInstance().runThroughTake(0.7);
        }
        // Shooter.getInstance().setShooterRPM(targetRPM);
    }

    @Override
    public void end(boolean interupted) {
        Shooter.getInstance().stopShooter();
        ThroughTake.getInstance().stopThroughTake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
