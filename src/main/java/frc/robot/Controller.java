package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.LimelightHelpers;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ThruTakeToShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ThroughTake;

public class Controller extends SubsystemBase{
    Joystick m_joystick1 = new Joystick(0);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    XboxController m_controller = new XboxController(2);

    ThruTakeToShooter m_thruTakeToShooter = new ThruTakeToShooter(ThroughTake.getInstance(), Shooter.getInstance());

    // values will be between 0 and 1 in this map
    private double[] PowerMap =
    {
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0.1,0.1,0.1,0.15,0.15,
        0.15,0.15,0.15,0.15,0.2,0.2,0.2,0.2,0.2,0.2,
        0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.3,0.3,
        0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.4,0.4,0.4,
        0.4,0.4,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,
        0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,
        0.6,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,
        0.7,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,
        0.9,0.9,0.9,0.9,0.9,1,1,1,1,1,1
    };
    private double ReturnValueFromMap(double index) {
        return index < 0 ? -PowerMap[(int)(-(index*100))] : PowerMap[(int)(index*100)];
    }
    private double setSpeed() {
        if (m_joystick1.getRawButton(1) == true) {
            return 2.0; // 9.0
        }
        else {
            return 15.0; // 2.0
        }
    }
    
    public Controller (){
        DriveTrain.getInstance().setDefaultCommand(new RunCommand(
              () -> DriveTrain.getInstance().drive(
                  
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                //   (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * 3.25,
                  (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * setSpeed(),
                  true, true),
              DriveTrain.getInstance()));
        //buttonBoard.SetupButtons();
        configureButtonBindings();
    }

    //configures all buttons
    private void configureButtonBindings(){
        new JoystickButton(m_joystick2, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().setX(),
                DriveTrain.getInstance()));

        // This button for the DRIVER will zero the gyro's angle
        new JoystickButton(m_joystick1, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().zeroHeading(),
                DriveTrain.getInstance()));

        new JoystickButton(m_joystick1, 3) 
            .toggleOnTrue(
                m_thruTakeToShooter
            );
               
        /* 
        new JoystickButton(m_joystick1, 3)
            .toggleOnTrue(
                new AutoAlign(DriveTrain.getInstance(), "limelight-two"));
        
        new JoystickButton(m_joystick1, 4)
            .toggleOnTrue(
                new AutoAlign(DriveTrain.getInstance(), "limelight-three"));
        */
        new JoystickButton(m_joystick1, 5)
            .toggleOnTrue(
                new InstantCommand(()-> 
                Shooter.getInstance().setShooterSpeed(0.7)));

        // actual subsystem button bindings
        new JoystickButton(m_joystick2, 3)
            .whileTrue(new RunCommand(
                () -> Intake.getInstance().moveIntakeUp(),
                Intake.getInstance()));

        new JoystickButton(m_joystick2, 4)
            .whileTrue(new RunCommand(
                () -> Intake.getInstance().moveIntakeDown(), 
                Intake.getInstance()));

        new JoystickButton(m_joystick2, 5)
            .whileTrue(new RunCommand(
                () -> Climber.getInstance().runClimberMotors(0.5),
                Climber.getInstance()));

        new JoystickButton(m_joystick2, 6)
            .whileTrue(new RunCommand(
                () -> Climber.getInstance().motionMagicClimberUp(),
                Climber.getInstance()));

        new JoystickButton(m_joystick2, 7)
            .whileTrue(new RunCommand(
                () -> Climber.getInstance().motionMagicClimberDown(),
                Climber.getInstance()));

        // new JoystickButton(m_joystick1, 5)
        //     .onTrue(DriveTrain.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        // new JoystickButton(m_joystick1, 6)
        //     .onTrue(DriveTrain.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // new JoystickButton(m_joystick2, 5)
        //     .onTrue(DriveTrain.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));

        // new JoystickButton(m_joystick2, 6)
        //     .onTrue(DriveTrain.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void periodic() {
        //   LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(LimelightConstants.limelight_three);
        //   System.out.println(detections);
    }
}
