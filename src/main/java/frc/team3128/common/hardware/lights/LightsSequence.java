package frc.team3128.common.hardware.lights;

import java.util.ArrayList;

/**
 * Class which represents a sequence of steps for the lights to display
 * 
 * @author Jamie
 *
 */
public class LightsSequence {
	ArrayList<Step> sequenceSteps;

	private boolean repeat;

	public boolean shouldRepeat() {
		return repeat;
	}

	public void setRepeat(boolean repeat) {
		this.repeat = repeat;
	}

	public static class Step {
		public LightsColor getColor() {
			return color;
		}

		public long getTimeInMillis() {
			return milliseconds;
		}

		public boolean fadeToNext() {
			return fadeToNext;
		}

		private LightsColor color;
		private long milliseconds;
		private boolean fadeToNext;

		/**
		 * Create a step in a sequence of lights from its color and duration
		 * 
		 * @param color
		 * @param milliseconds how long the color should display for. If this is zero,
		 *                     then the step will be skipped and the sequence will
		 *                     immediately transition into the next step
		 * @param fadeToNext
		 */
		public Step(LightsColor color, long milliseconds, boolean fadeToNext) {
			this.color = color;
			this.milliseconds = milliseconds;
			this.fadeToNext = fadeToNext;
		}

	}

	public LightsSequence() {
		sequenceSteps = new ArrayList<Step>();
	}

	/**
	 * Add a step to the lights sequence.
	 * 
	 * @param stepToAdd
	 */
	public void addStep(Step stepToAdd) {
		sequenceSteps.add(stepToAdd);
	}

}
