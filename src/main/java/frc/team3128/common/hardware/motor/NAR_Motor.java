package frc.team3128.common.hardware.motor;

public enum NAR_Motor {
    PRO_775(18730, 0.7, 0.71, 134);

    protected double freeSpeedRPM;
    protected double freeCurrentAmps;
    protected double stallCurrentAmps;
    protected double stallTorqueNM;

    NAR_Motor(double freeSpeedRPM, double freeCurrentAmps, double stallCurrentAmps, double stallTorqueNM) {
        this.freeSpeedRPM = freeSpeedRPM;
        this.freeCurrentAmps = freeCurrentAmps;
        this.stallCurrentAmps = stallCurrentAmps;
        this.stallTorqueNM = stallTorqueNM;
    }

    public double getFreeSpeedRPM() {
        return freeSpeedRPM;
    }

    public double getFreeCurrentAmps() {
        return freeCurrentAmps;
    } 

    public double getStallCurrentAmps() {
        return stallCurrentAmps;
    }

    public double getStallTorqueNM() {
        return stallTorqueNM;
    }
}