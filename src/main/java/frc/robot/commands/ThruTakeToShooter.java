package frc.robot.commands;

import frc.robot.subsystems.ThroughTake;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;

class ThruTakeToShooter  {
  private final ThroughTake take;
  private final Shooter shooter;

  public ThruTakeToShooter(ThroughTake take, Shooter shooter) {
    this.take = take;
    this.shooter = shooter;
  }

public void start() {
    double speed = shooter.getSpeed();
    shooter.setShooterSpeed(speed);
    Timer.delay(1);
    take.runThroughTake(); 
  }


  public void end() {
    shooter.setShooterSpeed(0);
    take.stopThroughTake();
  }
}