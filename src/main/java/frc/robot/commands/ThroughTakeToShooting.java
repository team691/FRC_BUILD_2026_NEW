package frc.robot.commands;

import frc.robot.subsystems.ThroughTake;
import frc.robot.subsystems.Shooter;

public class ThroughTakeToShooting {
    ThroughTake throughTake = new ThroughTake();
    Shooter shooter = new Shooter();

    public ThroughTakeToShooting() {
        throughTake.runThroughTake();
        
    }


}