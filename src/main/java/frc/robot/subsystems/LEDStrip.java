package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private static final LEDStrip m_instance = new LEDStrip();
    public static LEDStrip getInstance() { return m_instance; }

    private final PWM m_blinkin = new PWM(0);

    private LEDStrip() {}   // private constructor prevents external `new`

    public void Red()   { m_blinkin.setPulseTimeMicroseconds(1805); }
    public void Green() { m_blinkin.setPulseTimeMicroseconds(1885); }
    public void Blue()  { m_blinkin.setPulseTimeMicroseconds(1935); }
    public void Stuck() { m_blinkin.setPulseTimeMicroseconds(1445); }
    public void Shoot() { m_blinkin.setPulseTimeMicroseconds(1285); }
}