package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AlignToShoot;
import frc.robot.constants.Constants.*;

public class Climber extends SubsystemBase {
    TalonFX climberMotor1;
    TalonFX climberMotor2;

    TalonFXConfiguration climberMotionConfigs;

    // test these rotations
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    private static final Climber m_climber = new Climber();
    public static Climber getInstance() {return m_climber;}

    public Climber () {
        climberMotor1 = new TalonFX(ClimberConstants.climberTalonDeviceIdMotor1);
        climberMotor2 = new TalonFX(ClimberConstants.climberTalonDeviceIdMotor2);

        climberMotionConfigs = new TalonFXConfiguration();
        var slot0 = climberMotionConfigs.Slot0;
        slot0.kS = ClimberConstants.talonClimberConfigkS;
        slot0.kV = ClimberConstants.talonClimberConfigkV;
        slot0.kA = ClimberConstants.talonClimberConfigkA;
        slot0.kP = ClimberConstants.talonClimberConfigkP;
        slot0.kI = ClimberConstants.talonClimberConfigkI;
        slot0.kD = ClimberConstants.talonClimberConfigkD;

        var motionMagiConfiguration = climberMotionConfigs.MotionMagic;
        motionMagiConfiguration.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagiConfiguration.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagiConfiguration.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // config sets
        climberMotor1.getConfigurator().apply(climberMotionConfigs);
        climberMotor2.getConfigurator().apply(climberMotionConfigs);
    }

    public void runClimberMotors(double speed) {
        climberMotor1.set(speed);
        climberMotor2.set(speed);
    }

    public void motionMagicClimberUp() {
        //ClimberTalonMotor
        //ClimberTalonMotor2
        climberMotor1.setControl(positionRequest.withPosition(500));
        climberMotor2.setControl(positionRequest.withPosition(500));
    }

    public void motionMagicClimberDown() {
        climberMotor1.setControl(positionRequest.withPosition(0));
        climberMotor2.setControl(positionRequest.withPosition(0));
    }
}
