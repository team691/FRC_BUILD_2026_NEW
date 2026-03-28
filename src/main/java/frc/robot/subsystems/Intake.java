package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    // Timed move settings (configurable)
    private double moveUpSpeed = 1.0;
    private double moveDownSpeed = -0.6;
    private double moveUpDurationSec = 0.75;
    private double moveDownDurationSec = 0.75;

    private enum TimedMoveState { IDLE, MOVING_UP, MOVING_DOWN }
    private TimedMoveState timedMoveState = TimedMoveState.IDLE;
    private final Timer moveTimer = new Timer();


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
        intakeTalonMotor.set(moveUpSpeed);
    }
   
    public void moveIntakeDown() {
        intakeTalonMotor.set(moveDownSpeed);
        DogLog.log("Intake", "Intake down");
        intakeTalonMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void runIntake(){
        intakeNeoMotor.set(0.85);
        DogLog.log("Intake", "Intake running");
    }

    public void stopIntake() {
        intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake stopped");
    }

    public void stopElevatorIntake() {
        intakeTalonMotor.set(0);
        // intakeNeoMotor.set(0);
        intakeTalonMotor.setNeutralMode(NeutralModeValue.Brake);
        timedMoveState = TimedMoveState.IDLE;
        moveTimer.stop();
        moveTimer.reset();
    }

    @Override
    public void periodic() {
        switch (timedMoveState) {
            case MOVING_UP:
                if (moveTimer.hasElapsed(moveUpDurationSec)) {
                    stopElevatorIntake();
                }
                break;
            case MOVING_DOWN:
                if (moveTimer.hasElapsed(moveDownDurationSec)) {
                    stopElevatorIntake();
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
}