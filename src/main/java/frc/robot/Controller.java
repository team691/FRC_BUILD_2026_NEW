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
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.utils.LimelightHelpers;
// import frc.robot.commands.AutoAlign;
import frc.robot.commands.ThruTakeToShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ThroughTake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;

public class Controller extends SubsystemBase{
    Joystick m_joystick1 = new Joystick(0);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    XboxController m_controller = new XboxController(2);
    double shooterSpeedPercent = 0.845;
    PhotonCamera mustyCamera = new PhotonCamera(getName());
    double RPMshooter  = 2300; // 4500
    // ThruTakeToShooter m_thruTakeToShooter = new ThruTakeToShooter(ThroughTake.getInstance(), Shooter.getInstance());
    

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
            return 3.0; // 8.0
        }
        else {
            return 10.0; // 28.0
        }
    }
    
    public Controller (){
        DriveTrain.getInstance().setDefaultCommand(new RunCommand(
              () -> DriveTrain.getInstance().drive(
                  
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick2.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick2.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                  (-MathUtil.applyDeadband(m_joystick1.getZ(), OIConstants.kDriveDeadband)) * 10,
                //   (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * setSpeed(),
                  true, true),
              DriveTrain.getInstance()));
        configureButtonBindings();
        // SmartDashboard.putNumber("Shooter RPM", RPMshooter);
    }

    private void configureButtonBindings() {
        new JoystickButton(m_joystick2, 4)
            .whileTrue(
                new RunCommand(
                    () -> ThroughTake.getInstance().runThroughTake(0.6),
                    ThroughTake.getInstance()
                ).finallyDo((interrupted) -> ThroughTake.getInstance().stopThroughTake())
            );

        new JoystickButton(m_joystick2, 3)
            .whileTrue(
                new RunCommand(
                    () -> ThroughTake.getInstance().runThroughTake(-0.5),
                    ThroughTake.getInstance()
                ).finallyDo((interrupted) -> ThroughTake.getInstance().stopThroughTake())
            );

        // This button for the DRIVER will zero the gyro's angle
        new JoystickButton(m_joystick2, 12)
            .whileTrue(new InstantCommand( // test with InstantCommand
                () -> DriveTrain.getInstance().zeroHeading(),
                DriveTrain.getInstance()));
                
        new JoystickButton(m_joystick1, 12)
            .whileTrue(new RunCommand(
                () -> DriveTrain.getInstance().setX(),
                DriveTrain.getInstance()));

        new JoystickButton(m_joystick1, 2)
            .whileTrue(
                new RunCommand(()-> 
                Intake.getInstance().runIntake()))
            .whileFalse(
                new RunCommand(() ->
                Intake.getInstance().stopIntake()));

        new JoystickButton(m_joystick1, 5)
            .whileTrue(
                new RunCommand(
                    () -> Intake.getInstance().moveIntakeUp(),
                    Intake.getInstance()
                )
            )
            .whileFalse(
                new RunCommand(
                    () -> Intake.getInstance().stopElevatorIntake(),
                    Intake.getInstance()
                )
            );

        new JoystickButton(m_joystick1, 6)
            .whileTrue(
                new RunCommand(
                    () -> Intake.getInstance().moveIntakeDown(),
                    Intake.getInstance()
                )
            )
            .whileFalse(
                new RunCommand(
                    () -> Intake.getInstance().stopElevatorIntake(),
                    Intake.getInstance()
                )
            );

        new JoystickButton(m_joystick1, 3)
            .whileTrue(
                new RunCommand(()-> 
                Shooter.getInstance().setShooterRPM(RPMshooter)))
            .whileFalse(
                new RunCommand(() ->
                Shooter.getInstance().stopShooter()));
        
        new JoystickButton(m_joystick1, 4)
            .whileTrue(
                new RunCommand(
                    () -> Shooter.getInstance().setShooterSpeed(-0.75),
                    Shooter.getInstance()
                ).finallyDo((interrupted) -> Shooter.getInstance().stopShooter()));

        
        // new JoystickButton(m_joystick2, 4)
        //     .toggleOnTrue(
        //         new RunCommand(
        //             () -> Shooter.getInstance().setShooterRPM(RPMshooter),
        //             Shooter.getInstance()
        //         )
        //         .finallyDo((interrupted) -> Shooter.getInstance().stopShooter()));

        // new JoystickButton(m_joystick2, 9)
        //     .onTrue(new InstantCommand(() -> RPMshooter += 100));

        // new JoystickButton(m_joystick2, 10)
        //     .whileTrue(new RunCommand(() -> Shooter.getInstance().setShooterRPM(RPMshooter)));
            
        // new JoystickButton(m_joystick2, 11)
        //     .onTrue(new InstantCommand(() -> RPMshooter -= 100));
    }

    @Override
    public void periodic() {
        //   LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(LimelightConstants.limelight_three);
        //   System.out.println(detections);
        RPMshooter = SmartDashboard.getNumber("Shooter RPM", 4500);
    }
}