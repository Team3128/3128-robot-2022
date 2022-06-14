package frc.team3128.common.hardware.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Software wrapper to obtain data from and send data to the physical Limelight.
 * 
 * LIMELIGHT CONVENTIONS: - When the target is right of the vertical centerline,
 * tx is positive. - When the target is above the horizontal centerline, ty is
 * positive.
 * 
 * 
 * @author Adham Elarabawy, Mason Holst, Jude Lifset, Mika Okamoto
 *
 */
public class Limelight {
    public String hostname;
    public double cameraAngle;
    public double cameraHeight;
    public double targetWidth;

    public double frontDistance;

    public NetworkTable limelightTable;

    /**
     * 
     * @param cameraAngle   - The vertical angle of the limelight
     * @param cameraHeight  - The height off of the ground of the limelight
     * @param frontDistance - The distance between the front of the robot and the
     *                      Limelight
     * @param targetWidth   - The width of the target
     */
    public Limelight(String hostname, double cameraAngle, double cameraHeight, double frontDistance) {
        this.hostname = hostname;

        this.cameraAngle = cameraAngle;
        this.cameraHeight = cameraHeight;

        this.frontDistance = frontDistance;

        limelightTable = NetworkTableInstance.getDefault().getTable(hostname);
    }

    /**
     * Gets the median value of the data value in a certain key output by the
     * Limelight.
     * Switched from average to median to attempt to mitigate outliers
     * 
     * @param key        - the LimelightKey corresponding to the desired value.
     * @param numSamples - how many samples of the value to average out.
     * @return
     */
    public double getValue(LimelightKey key) {
        return limelightTable.getEntry(key.getKey()).getDouble(0.0);
    }

    public double getValueAverage(LimelightKey key, int numSamples) {
        double runningTotal = 0;

        for (int i = 0; i < numSamples; i++) {
            runningTotal += getValue(key);
        }

        return runningTotal / numSamples;
    }

    /**
     * Checks to see if the Limelight has a valid target
     */
    public boolean hasValidTarget() {
        return getValue(LimelightKey.VALID_TARGET) > 0.99;
    }

    public double calculateDistToTopTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        double tx = getValue(LimelightKey.HORIZONTAL_OFFSET) * Math.PI / 180;
        return (targetHeight - cameraHeight) / (Math.tan(ty + cameraAngle) * Math.cos(tx)) - frontDistance;
    }
    
    public double calculateDistToGroundTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET) * Math.PI / 180;
        return (-targetHeight + cameraHeight) * Math.tan(ty + cameraAngle) - frontDistance;
    }

    public void setLEDMode(LEDMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.getLEDMode());
    }

    public void setStreamMode(StreamMode mode) {
        limelightTable.getEntry("stream").setNumber(mode.getStream());
    }

    public void setPipeline(Pipeline pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline.getPipeline());
    }

      /**
     * Set the limelight on the dashboard
     */
    public double getSelectedPipeline() {
        return limelightTable.getEntry("pipeline").getDouble(0);
    }
}
