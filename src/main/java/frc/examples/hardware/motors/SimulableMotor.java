package frc.examples.hardware.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SpeedController;
import frc.examples.common.Simulable;

public abstract class SimulableMotor<T extends SpeedController> extends Simulable {
    protected int deviceNumber;
    protected T motorController;
    protected double freeSpeedRPM;
    protected double freeCurrentAmps;
    protected double stallCurrentAmps;
    protected double stallTorqueNM;

    public SimulableMotor(int deviceNumber) {
        super();
        this.deviceNumber = deviceNumber;
    }

    @Override
    public void updateSimulation(double timeStep) {
        // TODO Logic for updating the simulation
    }

    public abstract void set(double value);
    public abstract void set(ControlMode controlMode, double value);

    public void stop() {
        motorController.stopMotor();
    }

    public T getMotorController() {
        return (T) motorController;
    } 

    public double getFreeSpeedRPM() {
        return freeSpeedRPM;
    }

    public double getFreeCurrentAmps() {
        return freeCurrentAmps;
    }

    public double getStallCurrentAmps () {
        return stallCurrentAmps;
    }

    public double getStallTorqueNM() {
        return stallTorqueNM;
    }
}
