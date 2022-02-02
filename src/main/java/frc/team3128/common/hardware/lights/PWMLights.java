package frc.team3128.common.hardware.lights;

import frc.team3128.common.hardware.lights.LightsSequence.Step;
import frc.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;

/**
 * Controls a RGB light strip through three PWM outputs, one for each color.
 * 
 * It can also fade the lights on and off.
 * 
 * @author Jamie
 *
 */
public class PWMLights {
	PWM redLights;
	PWM greenLights;
	PWM blueLights;

	Thread sequenceThread;

	/**
	 * Construct a PWMLights object from the three PWM ports on the roboRIO it is
	 * attached to.
	 * 
	 * @param redChannel
	 * @param greenChannel
	 * @param blueChannel
	 */
	public PWMLights(int redChannel, int greenChannel, int blueChannel) {
		redLights = new PWM(redChannel);
		greenLights = new PWM(greenChannel);
		blueLights = new PWM(blueChannel);

		// turn off legacy period scaling
		// gee, thanks WPI for not documenting this ANYWHERE!
		redLights.setPeriodMultiplier(PeriodMultiplier.k1X);
		greenLights.setPeriodMultiplier(PeriodMultiplier.k1X);
		blueLights.setPeriodMultiplier(PeriodMultiplier.k1X);
	}

	/**
	 * Set the color of the lights in RGB.
	 * 
	 * If the fader thread is running, calling this function will stop it.
	 * 
	 * @param r
	 * @param g
	 * @param b
	 */
	// there are no short literals in Java, so the function takes ints for
	// convenience.
	public void setColor(LightsColor color) {
		// Make sure that the worker thread is not setting these values at the same
		// time.
		shutDownSequenceThread();

		redLights.setRaw(color.getR());
		greenLights.setRaw(color.getG());
		blueLights.setRaw(color.getB());
	}

	/**
	 * Turn the lights off entirely.
	 */
	public void setOff() {
		setColor(LightsColor.new4Bit(0, 0, 0));
	}

	/**
	 * Start the fader thread, which pulses the lights on and off.
	 * 
	 * @param color
	 */
	public void setFader(LightsColor color) {
		shutDownSequenceThread();

		// make a sequence which fades to black and back.
		LightsSequence faderSequence = new LightsSequence();
		faderSequence.setRepeat(true);
		faderSequence.addStep(new Step(color, 0, true));

		// we need to not ever turn on channels which are zero in the starting color to
		// avoid weirdness
		// but if we fade all the way to zero then the lights flash off for a moment
		// hence the ternary operators
		LightsColor darkColor = LightsColor.new11Bit(color.getR() > 0 ? 2 : 0, color.getG() > 0 ? 2 : 0,
				color.getB() > 0 ? 2 : 0);

		faderSequence.addStep(new Step(darkColor, 0, true));

		executeSequence(faderSequence);
	}

	/**
	 * Stop the fader thread running, leaving the lights wherever they happen to be.
	 * 
	 * If the fader thread is not running, does nothing.
	 * 
	 * Called by setColor() and setOff()
	 */
	public void shutDownSequenceThread() {
		if (sequenceThread != null && sequenceThread.isAlive()) {
			sequenceThread.interrupt();
			try {
				sequenceThread.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Function run from the worker thread to fade the lights. It takes the base
	 * color the lights should fade from in 11 bit form as the first parameter.
	 *
	 * @param time       how long the fade operation should take.
	 * @param resolution how many steps per second to do the fade in.
	 */
	private void faderLoop(LightsColor originalColor, LightsColor newColor, long time, int resolution) {
		// Log.debug("PWMLights", "Fader Loop Starting");

		double currentR = originalColor.getR();
		double currentG = originalColor.getG();
		double currentB = originalColor.getB();

		int numSteps = (int) Math.ceil((time / 1000.0) * resolution);
		int sleepTime = (int) Math.ceil(1000.0 / resolution);

		int differenceR = newColor.getR() - originalColor.getR();
		double incrementR = differenceR / (double) numSteps;

		int differenceG = newColor.getG() - originalColor.getG();
		double incrementG = differenceG / (double) numSteps;

		int differenceB = newColor.getB() - originalColor.getB();
		double incrementB = differenceB / (double) numSteps;

		// Log.debug("PWMLights", "numSteps: " + numSteps + " incrementR: " + incrementR
		// + " incrementG: " + incrementG + " incrementB: " + incrementB);

		for (int currentStep = 0; currentStep < numSteps; ++currentStep) {
			currentR += incrementR;
			currentG += incrementG;
			currentB += incrementB;

			// actually set the values
			redLights.setRaw((int) currentR);
			greenLights.setRaw((int) currentG);
			blueLights.setRaw((int) currentB);

			// Log.debug("PWMLights", String.format("R: %.02f, G: %.02f, B: %.02f",
			// currentR, currentG, currentB));

			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				// Log.debug("PWMLights", "Fader Thread Shutting Down");
			}
		}
	}

	/**
	 * Execute a sequence of lights changes.
	 * 
	 * @param sequence
	 */
	public void executeSequence(LightsSequence sequence) {

		shutDownSequenceThread();
		Log.debug("PWMLights", "Executing lights sequence.");

		sequenceThread = new Thread(() -> {
			do {
				for (int counter = 0; counter < sequence.sequenceSteps.size(); ++counter) {
					Step currentStep = sequence.sequenceSteps.get(counter);

					// avoid calling setColor() because it shuts down the thread
					redLights.setRaw(currentStep.getColor().getR());
					greenLights.setRaw(currentStep.getColor().getG());
					blueLights.setRaw(currentStep.getColor().getB());

					if (currentStep.getTimeInMillis() > 0) {
						try {
							Thread.sleep(currentStep.getTimeInMillis());
						} catch (InterruptedException ex) {
							return;
						}
					}

					if (currentStep.fadeToNext()) {
						// wrap around to the first step if this is the last step
						LightsColor newColor = sequence.sequenceSteps
								.get(counter == sequence.sequenceSteps.size() - 1 ? 0 : counter + 1).getColor();
						faderLoop(currentStep.getColor(), newColor, 2000, 30);
					}
				}
			} while (sequence.shouldRepeat());

			Log.debug("PWMLights", "Finished lights sequence.");
		});

		sequenceThread.start();
	}
}
