//Warning! this is just temple code that I made, we need to actually apply this to the subsystems when they are done

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Buttons {
    private final TalonFX tell = new TalonFX(1);
    private final Joystick Joy = new Joystick(0);
    private final JoystickButton Joy1 = new JoystickButton(Joy, 1);
    private final JoystickButton Joy2 = new JoystickButton(Joy, 2);
    private final JoystickButton Joy3 = new JoystickButton(Joy, 3);
    public void Buttonbinding() {
        Joy1.whileTrue(new InstantCommand(() -> tell.set(0.25)));
        Joy2.whileTrue(new InstantCommand(() -> tell.set(0.50)));
        Joy3.whileTrue(new InstantCommand(() -> tell.set(0.75)));
    }
}
