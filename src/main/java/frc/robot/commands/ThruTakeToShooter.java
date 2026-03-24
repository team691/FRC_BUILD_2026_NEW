package frc.robot.commands;

import frc.robot.subsystems.ThroughTake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

// TODO: run thrutake + shooter in reverse when amps >30amps, run throughtake + shooter forward once amps <20amp avg

public class ThruTakeToShooter extends Command {
  private final ThroughTake m_thrutake;
  private final Shooter shooter;

  public ThruTakeToShooter(ThroughTake m_thrutake, Shooter m_shooter) {
    this.m_thrutake = m_thrutake;
    this.shooter = m_shooter;
  }

  public static void ToggleShooterOn() {
        ThroughTake.getInstance().runThroughTake(0.5);
        Commands.waitSeconds(0.25);
        Shooter.getInstance().setShooterSpeed(0.2);
    }
    public static void ToggleShooterOff() {
        ThroughTake.getInstance().stopThroughTake();
        Shooter.getInstance().setShooterSpeed(0);
    }

public void execute() {
    // shooter.getSpeed();
    // double m_speed = shooter.getSpeed();
    // System.out.println("speed "+m_speed);
    double m_speed = 0.9;
    shooter.setShooterSpeed(m_speed);
    // Timer.delay(1.0);
    Commands.waitSeconds(0.5);
    // m_timer.delay(0.7);
    m_thrutake.runThroughTake(1.0);
    // Timer.delay(0.5);
  }
public void executeBackwards() {
    // shooter.getSpeed();
    // double m_speed = shooter.getSpeed();
    // System.out.println("speed "+m_speed);
    double m_speed = -0.3;
    shooter.setShooterSpeed(m_speed);
    // Timer.delay(1.0);
    // m_timer.delay(0.7);
    m_thrutake.runThroughTake(-1.0);
    // Timer.delay(0.5);
  }

  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.setShooterSpeed(0);
      m_thrutake.stopThroughTake();
    }
}}