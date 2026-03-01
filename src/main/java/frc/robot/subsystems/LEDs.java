package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private final int kLEDLength = 36; // number of leds 

    public LEDs(int LEDPort) {
        m_led = new AddressableLED(LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(kLEDLength);
        m_led.start();
    }
    public void setAllLEDsToColor(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
    }
    public void setLEDColor(int index, Color color) {
        m_ledBuffer.setLED(index, color);
    }
    
    
}
