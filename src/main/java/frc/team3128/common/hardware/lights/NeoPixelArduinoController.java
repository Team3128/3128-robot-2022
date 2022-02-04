package frc.team3128.common.hardware.lights;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;

/**
 * Controller for the NeoPixel light strip. More accurately, it sends data using
 * digital IO ports on the roboRIO, which is then processed by the Arduino Uno
 * running the corresponding NeoPixelController script.
 * 
 * This class should be extended within each year's robot code in order to have
 * methods defined for specific usage within that years game, such as sustained
 * green blinking or momentary blue blinking.
 * 
 * @author Ronak
 *
 */
public class NeoPixelArduinoController {
	private final int idleData = 250;

	PWM dataPort;
	DigitalInput confirmationPort;

	/**
	 * Creates a new light controller that is capable of sending single integer
	 * values to the Arduino. Furthermore, you can choose to maintain a data value
	 * or stop sending that data value once the Uno has confirmed receiving the
	 * value.
	 * 
	 * @param confirmationPort - The digital input port at which the Arduino will
	 *                         set to true upon receiving the data if the data sent
	 *                         is defined as non-sustaining.
	 * @param dataPort         - The PWM output to send the PWM signal representing
	 *                         the Arduino command from
	 */
	public NeoPixelArduinoController(DigitalInput confirmationPort, PWM dataPort) {
		this.dataPort = dataPort;
		this.confirmationPort = confirmationPort;
	}

	/**
	 * Clears the data ports and sends over new data. If sustained is true, the data
	 * will simply be set, and it will not be cleared until the ports are manually
	 * cleared or new data is chosen to be sent. If sustained is false, the program
	 * will await the confirmation port, signifying the Arduino has received the
	 * data, and then clear the ports.
	 * 
	 * @param data      - The PWM value to be sent to the Arduino.
	 * @param sustained - Whether or not the data should maintain or be erased after
	 *                  being received.
	 */
	public void sendData(int data, boolean sustained) {
		sendCommand(data);

		if (!sustained) {
			Thread waitForConfirmation = new Thread(() -> {
				while (!confirmationPort.get()) {
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}

				sendCommand(idleData);
			});

			waitForConfirmation.start();
		}

	}

	public synchronized void sendCommand(int pwm) {
		dataPort.setRaw(pwm);
	}
}
