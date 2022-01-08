package frc.team3128.common.hardware.limelight;

public enum LimelightKey {
    VALID_TARGET("tv"),
    HORIZONTAL_OFFSET("tx"),
    VERTICAL_OFFSET("ty"),
    AREA("ta"),
    SKEW("ts"),
    LATENCY("tl"),
    FITTED_SHORT("tshort"),
    FITTED_LONG("tlong"),
    LENGTH_HORIZONTAL("thor"),
    LENGTH_VERTICAL("tvert"),
    PIPELINE("getpipe");

    private String key;
    private LimelightKey(String key) {
        this.key = key;
    }

    public String getKey() {
        return key;
    }

    public String toString() {
        return getKey();
    }
}