package frc.team3128.common.hardware.limelight;

import java.util.LinkedHashMap;
import java.util.Map;

public class LimelightData extends LinkedHashMap<String, Double>{

    private String csvString;

    public LimelightData() {
        super();
        for (String valueKey : LimelightConstants.VALUE_KEYS) {
            put(valueKey, 0.0);
        }
        for (String valueKey : LimelightConstants.VALUE_KEYS_PNP) {
            put(valueKey, 0.0);
        }
    }

    public String toString() {

        csvString = "";

        for (Map.Entry<String, Double> entry : entrySet()) {
            csvString += (Double.toString(entry.getValue()) + ", ");
        }

        csvString += "\n";

        return csvString;
    }
}