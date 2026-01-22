package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Limelight;
import frc.robot.commands.AutoAlign;

public class Controller extends SubsystemBase{

    Joystick m_joystick1 = new Joystick(0);
    Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
    XboxController m_controller = new XboxController(2);
    // Limelight m_lime = new Limelight(DriveTrain.getInstance());

    boolean shouldRunBelt = true;
    boolean isPressed = false;
    boolean isBeltOn = false;
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
            return 8.0; // 2.0
        }
    }
    public Controller (){
        DriveTrain.getInstance().setDefaultCommand(new RunCommand(
              () -> DriveTrain.getInstance().drive(
                  
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getY(), OIConstants.kDriveDeadband)) * setSpeed() , //m_operator.getRawAxis(3)
                  ReturnValueFromMap(MathUtil.applyDeadband(m_joystick1.getX(), OIConstants.kDriveDeadband)) * setSpeed() , // * m_sonar.getSpeed(sonarOn)
                  (-MathUtil.applyDeadband(m_joystick2.getZ(), OIConstants.kDriveDeadband)) * 3.25,
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
                new AutoAlign(DriveTrain.getInstance(), "limelight-two"));

        new JoystickButton(m_joystick1, 4)
            .toggleOnTrue(
                new AutoAlign(DriveTrain.getInstance(), "limelight-three"));

        new JoystickButton(m_joystick1, 5)
            .whileTrue(DriveTrain.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        new JoystickButton(m_joystick1, 6)
            .whileTrue(DriveTrain.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void periodic() {

    }
}
