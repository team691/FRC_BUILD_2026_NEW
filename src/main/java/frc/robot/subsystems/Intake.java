package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;


import org.littletonrobotics.junction.Logger;


import dev.doglog.DogLog;
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

    MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    public Intake () {
        intakeTalonMotor = new TalonFX(IntakeConstants.ballIntakeTalonId);
        intakeNeoMotor = new SparkMax(IntakeConstants.moveIntakeDownId, IntakeConstants.intakeCanMotorType);
    }


    public void moveIntakeUp() {
        // intakeTalonMotor.set(0.3);
        stopIntake();
        intakeTalonMotor.setControl(positionRequest.withPosition(500));
        DogLog.log("Intake", "Intake up");
        DogLog.log("Intake", "talon current " + intakeTalonMotor.getSupplyCurrent());
        Elastic.sendNotification(IntakeChange);
    }
   
    public void moveIntakeDown() {
        runIntake();
        intakeTalonMotor.setControl(positionRequest.withPosition(0));
        // intakeTalonMotor.set(0);
        DogLog.log("Intake", "Intake down");
    }


    public void runIntake(){
        intakeNeoMotor.set(0.7);
        // intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake running");
    }

    public void stopIntake() {
        intakeNeoMotor.set(0);
        DogLog.log("Intake", "Intake stopped");
    }
   
}