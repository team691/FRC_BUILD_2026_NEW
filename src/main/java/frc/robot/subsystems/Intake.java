package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.*;

public class Intake extends SubsystemBase {
    // one neo for spin another kraken for up and down

    TalonFX intakeTalonMotor;
    SparkMax intakeNeoMotor;

    private static final Intake m_intake = new Intake();
    public static Intake getInstance() {return m_intake;}

    public Intake () {
        intakeTalonMotor = new TalonFX(IntakeConstants.shooterTalonDeviceId);
        intakeNeoMotor = new SparkMax(IntakeConstants.shooterCanDeviceId, IntakeConstants.shooterCanMotorType);
    }

    public void moveIntakeUp() {
        intakeTalonMotor.set(0.5);
        intakeTalonMotor.set(0);
    }
    
    public void moveIntakedown() {
        intakeTalonMotor.set(-0.5);
        intakeTalonMotor.set(0);
    }

    public void runIntake(){
        intakeNeoMotor.set(0.5);
        intakeNeoMotor.set(0);

    }
    
}

