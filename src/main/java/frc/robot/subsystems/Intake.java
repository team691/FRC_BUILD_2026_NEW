package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.NotificationLevel;


public class Intake extends SubsystemBase {
    // one neo for spin another kraken for up and down
    TalonFX intakeTalonMotor;
    SparkMax intakeNeoMotor;

    private static final Intake m_intake = new Intake();
    public static Intake getInstance() {return m_intake;}

    public final Notification IntakeChange = new Notification(NotificationLevel.INFO, "Intake Change", "Intake position has been changed.");

    final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    public static int rotationAmount = 20;

    public Intake () {
        intakeTalonMotor = new TalonFX(IntakeConstants.ballIntakeTalonId);
        intakeNeoMotor = new SparkMax(IntakeConstants.moveIntakeDownId, IntakeConstants.intakeCanMotorType);
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        intakeTalonMotor.getConfigurator().apply(talonFXConfigs);
    }


    public void moveIntakeUp() {
        // intakeTalonMotor.set(0.3);
        // stopIntake();
        intakeNeoMotor.set(0.5);
        intakeTalonMotor.setControl(positionRequest.withPosition(rotationAmount));
        DogLog.log("Intake", "Intake up");
        DogLog.log("Intake", "talon current " + intakeTalonMotor.getSupplyCurrent());
        Elastic.sendNotification(IntakeChange);
    }
   
    public void moveIntakeDown() {
        runIntake();
        intakeTalonMotor.setControl(positionRequest.withPosition(-rotationAmount));
        // intakeTalonMotor.set(0);
        DogLog.log("Intake", "Intake down");
    }

    public void runIntake(){
        intakeNeoMotor.set(0.6);
        // intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake running");
    }

    public void stopIntake() {
        intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake stopped");
    }
   
}