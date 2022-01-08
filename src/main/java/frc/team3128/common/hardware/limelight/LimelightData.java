package frc.team3128.common.hardware.limelight;

import java.util.LinkedHashMap;
import java.util.Map;

public class LimelightData {

    private LinkedHashMap<String, Double> values;
    private String csvString;

    public LimelightData() {
        // values = new HashMap<String, Double>();
        values = new LinkedHashMap<String, Double>();

        for (String valueKey : LimelightConstants.valueKeys) {
            values.put(valueKey, 0.0);
        }
        for (String valueKey : LimelightConstants.valueKeysPnP) {
            values.put(valueKey, 0.0);
        }
    }

    // Getters
    public double tx() {
        return values.get("tx");
    }

    public double ty() {
        return values.get("ty");
    }

    public double tv() {
        return values.get("tv");
    }

    public double shear() {
        return values.get("ts");
    }

    public double area() {
        return values.get("ta");
    }

    public double boxWidth() {
        return values.get("thor");
    }

    public double boxHeight() {
        return values.get("tvert");
    }

    public double fittedShort() {
        return values.get("tshort");
    }

    public double fittedLong() {
        return values.get("tlong");
    }

    public double x() {
        return values.get("x");
    }

    public double y() {
        return values.get("y");
    }

    public double z() {
        return values.get("z");
    }

    public double pitch() {
        return values.get("pitch");
    }

    public double yaw() {
        return values.get("yaw");
    }

    public double roll() {
        return values.get("roll");
    }

    // Setter
    public void set(String valueKey, double value) {
        values.put(valueKey, value);
    }

    public String toString() {

        csvString = "";

        for (Map.Entry<String, Double> entry : values.entrySet()) {
            csvString += (Double.toString(entry.getValue()) + ", ");
        }

        csvString += "\n";

        return csvString;
    }
}