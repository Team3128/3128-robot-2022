package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    
    private static LED instance;

    private AddressableLED leftLED; //, rightLED;
    private AddressableLEDBuffer leftBuffer; //, rightBuffer;

    private int len = 120;

    private int m_rainbowFirstPixelHue = 0;

    public static synchronized LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    public LED() {
        leftLED = new AddressableLED(8);
        // rightLED = new AddressableLED(9);

        leftBuffer = new AddressableLEDBuffer(len);
        // rightBuffer = new AddressableLEDBuffer(len);

        leftLED.setLength(leftBuffer.getLength());
        // rightLED.setLength(rightBuffer.getLength());
    
        leftLED.setData(leftBuffer);
        // rightLED.setData(rightBuffer);    

        leftLED.start();
        // rightLED.start();
    }

    private void rainbow() {
        for (int i = 0; i < len; i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / len)) % 180;
            leftBuffer.setHSV(i, hue, 255, 128);
            // rightBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;

        leftLED.setData(leftBuffer);
    }

    @Override
    public void periodic() {
        rainbow();
        leftLED.setData(leftBuffer);
        // rightLED.setData(rightBuffer);
    }

}
