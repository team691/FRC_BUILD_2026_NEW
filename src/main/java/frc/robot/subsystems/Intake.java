package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
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

    public static double rotationAmount = 0.01;

    // anti-gravity (small opposite-direction output) defaults
    private volatile double antiGravityPower = -0.1;             // small opposite-direction percent output
    private volatile double antiGravityDurationSec = 0.2;        // seconds to run anti-gravity output
    private final java.util.concurrent.atomic.AtomicBoolean antiGravityActive =
            new java.util.concurrent.atomic.AtomicBoolean(false);

    public Intake () {
        intakeTalonMotor = new TalonFX(IntakeConstants.ballIntakeTalonId);
        intakeNeoMotor = new SparkMax(IntakeConstants.moveIntakeDownId, IntakeConstants.intakeCanMotorType);
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 1.0; // 4.8 A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // 0.1 A velocity error of 1 rps results in 0.1 V output

        // TODO: add reverse spin when going down for intake to ensure it doesnt break every time

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80/50; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160/50; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600/50; // Target jerk of 1600 rps/s/s (0.1 seconds)

        intakeTalonMotor.getConfigurator().apply(talonFXConfigs);
    }


    public void moveIntakeUp() {
        // intakeTalonMotor.set(0.3);
        // stopIntake();
        stopIntake();
        intakeTalonMotor.setControl(positionRequest.withPosition(-rotationAmount));
        DogLog.log("Intake", "Intake up");
        DogLog.log("Intake", "talon current " + intakeTalonMotor.getSupplyCurrent());
        Elastic.sendNotification(IntakeChange);
    }
   
    public void moveIntakeDown() {
        runIntake();
        Timer time = new Timer();
        time.start();
        intakeTalonMotor.setControl(positionRequest.withPosition(rotationAmount));
        System.out.println("reached apex");
        // intakeTalonMotor.set(-1);
        System.out.println("antigravity on");
        if (time.hasElapsed(2)) {
            System.out.println("time stopped, antigravity of");
            time.stop();
            stopIntake();
        }
        DogLog.log("Intake", "Intake down");
    }

    // ...existing code...
    // public void moveIntakeDown() {
    //     runIntake();
    //     // Request motion magic to move down
    //     intakeTalonMotor.setControl(positionRequest.withPosition(rotationAmount));
    //     DogLog.log("Intake", "Intake down (motion request sent)");

    //     // Apply a small opposite-direction output for configurable time to slow descent (anti-gravity)
    //     if (antiGravityActive.compareAndSet(false, true)) {
    //         new Thread(() -> {
    //             try {
    //                 DogLog.log("Intake", "Anti-gravity hold start: power=" + antiGravityPower + " dur=" + antiGravityDurationSec);
    //                 // Apply small opposite-direction percent output
    //                 intakeTalonMotor.set(antiGravityPower);
    //                 Thread.sleep((long)(antiGravityDurationSec * 1000.0));
    //             } catch (InterruptedException e) {
    //                 Thread.currentThread().interrupt();
    //             } finally {
    //                 // Re-apply the position control so closed-loop resumes holding the target
    //                 intakeTalonMotor.setControl(positionRequest.withPosition(rotationAmount));
    //                 DogLog.log("Intake", "Anti-gravity hold end, resumed position control");
    //                 antiGravityActive.set(false);
    //             }
    //         }, "IntakeAntiGravity").start();
    //     } else {
    //         DogLog.log("Intake", "Anti-gravity already active, not starting another");
    //     }
    // }
// ...existing code...

    public void runIntake(){
        intakeTalonMotor.set(-0.1);
        intakeNeoMotor.set(1.0);
        // intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake running");
    }

    public void stopIntake() {
        intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake stopped");
    }
   
    // Adjustable anti-gravity parameters
    public void setAntiGravityPower(double power) {
        this.antiGravityPower = power;
    }

    public double getAntiGravityPower() {
        return this.antiGravityPower;
    }

    public void setAntiGravityDurationSec(double durationSec) {
        this.antiGravityDurationSec = durationSec;
    }

    public double getAntiGravityDurationSec() {
        return this.antiGravityDurationSec;
    }
}