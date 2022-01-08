package frc.team3128.common.hardware.limelight;

public enum StreamMode {
    BOTH(0),
    LIMELIGHT_CAMERA(1),
    DRIVER_CAMERA(2);

    private int stream;
    private StreamMode(int stream) {
        this.stream = stream;
    }

    public int getStream() {
        return stream;
    }
}