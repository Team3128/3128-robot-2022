package frc.team3128.common.hardware.limelight;

public enum Pipeline {
    DEFAULT(0),
    GRIP(1),
    ZOOM(2),
    CAM(3);

    private int pipeline;
    private Pipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    public int getPipeline() {
        return pipeline;
    }
}