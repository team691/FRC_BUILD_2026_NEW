package frc.robot.commands;

import frc.robot.subsystems.ThroughTake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ThruTakeToShooter extends Command {
  private final ThroughTake m_thrutake;
  private final Shooter shooter;

  public ThruTakeToShooter(ThroughTake m_thrutake, Shooter m_shooter) {
    this.m_thrutake = m_thrutake;
    this.shooter = m_shooter;
  }

public void execute() {
    shooter.getSpeed();
    // double m_speed = shooter.getSpeed();
    // System.out.println("speed "+m_speed);
    double m_speed = 0.8;
    shooter.setShooterSpeed(m_speed);
    Timer.delay(0.5);
    m_thrutake.runThroughTake(); 
  }

  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.setShooterSpeed(0);
      m_thrutake.stopThroughTake();
    }
}}