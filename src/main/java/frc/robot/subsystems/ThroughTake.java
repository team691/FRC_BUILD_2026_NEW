package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.*;
import frc.robot.constants.Constants.*;

public class ThroughTake extends SubsystemBase {
  private SparkMax throughTakeMotor;

  private static final ThroughTake m_thrutake = new ThroughTake();
  public static ThroughTake getInstance() {return m_thrutake;}

  public ThroughTake() {
    throughTakeMotor = new SparkMax(ThroughTakeConstants.throughTakeCanId, ThroughTakeConstants.throughTakeMotorType);
  }

  public void runThroughTake(double speed) {
    throughTakeMotor.set(speed);
    DogLog.log("ThroughTake", "ThroughTake running");
  }

  public void stopThroughTake() {
    throughTakeMotor.set(0);
    DogLog.log("ThroughTake", "ThroughTake stopped");
  }


}
