package frc.team3128.common.hardware.limelight;

import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Collections;

/**
 * Software wrapper to obtain data from and send data to the physical Limelight.
 * 
 * LIMELIGHT CONVENTIONS: - When the target is right of the vertical centerline,
 * tx is positive. - When the target is above the horizontal centerline, ty is
 * positive.
 * 
 * 
 * @author Adham Elarabawy, Mason Holst, Jude Lifset
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
    public double getValue(LimelightKey key, int numSamples) {
        ArrayList<Double> values = new ArrayList<Double>();
        // this is probably bad actually since to get median it has to be sorted, but numSamples should always be relatively small and this will cancel out weird outliers

        for (int i = 0; i < numSamples; i++) {
            values.add(limelightTable.getEntry(key.getKey()).getDouble(0.0));
        }

        Collections.sort(values);

        if (numSamples % 2 == 0) { // if even
            return (values.get(numSamples / 2) + values.get(numSamples / 2 - 1)) / 2;
        } else { // if odd
            return values.get((numSamples - 1) / 2);
        }
    }

    public double getValueAverage(LimelightKey key, int numSamples) {
        double runningTotal = 0;
        int count = 0;

        while (count <= numSamples) {
            runningTotal += limelightTable.getEntry(key.getKey()).getDouble(0.0);
            count += 1;
        }

        return runningTotal / numSamples;
    }

    /**
     * Checks to see if the Limelight has a valid target
     */
    public boolean hasValidTarget() {
        return getValueAverage(LimelightKey.VALID_TARGET, 1) > 0.99;
    }

    public double calculateDistToTopTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET, 5) * Math.PI / 180;
        double tx = getValue(LimelightKey.HORIZONTAL_OFFSET, 5) * Math.PI / 180;
        double dist = (targetHeight - cameraHeight) / (Math.tan(ty + cameraAngle) * Math.cos(tx)) - frontDistance;

        // double transformedDist = 19.4 + 0.243 * dist + 6.26e-3 * dist * dist;

        return dist;
    }
    
    public double calculateDistToGroundTarget(double targetHeight) {
        if (!hasValidTarget())
            return -1;
        double ty = getValue(LimelightKey.VERTICAL_OFFSET, 5) * Math.PI / 180;
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

    public LimelightData getValues(int numSamples) {
        LimelightData data = new LimelightData();
        double[] runningTotal;
        double[] camtranArray;
        int idx;
        int index = 0;
        runningTotal = new double[9];

        for (int a = 0; a < numSamples; a++) {

            Arrays.fill(runningTotal, 0.0);
            idx = 0;

            for (String valueKey : LimelightConstants.VALUE_KEYS) {
                runningTotal[idx] += limelightTable.getEntry(valueKey).getDouble(0.0);
                idx++;
            }
        }
        idx = 0;
        for (String valueKey : LimelightConstants.VALUE_KEYS) {
            data.set(valueKey, runningTotal[idx] / numSamples);
            idx++;
        }

        // load the camtranArray
        camtranArray = limelightTable.getEntry("camtran").getDoubleArray(new double[6]);
        // add each element in the array to the updated values for however many times
        // numSamples dictates(minus 1 because the array is initially loaded in with
        // values)
        for (int a = 0; a < numSamples - 1; a++) {
            for (int b = 0; b < camtranArray.length; b++) {
                camtranArray[b] += limelightTable.getEntry("camtran").getDoubleArray(new double[6])[b];
            }
        }
        for (String valueKey : LimelightConstants.VALUE_KEYS_PNP) {
            data.set(valueKey, camtranArray[index] / numSamples);
            index++;
        }

        return data;
    }
}
