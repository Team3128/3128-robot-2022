package frc.team3128.common.hardware.limelight;

public enum LEDMode {
    ON(3), OFF(1), BLINK(2);

    private int ledMode;

    private LEDMode(int ledMode) {
        this.ledMode = ledMode;
    }

    public int getLEDMode() {
        return ledMode;
    }
}