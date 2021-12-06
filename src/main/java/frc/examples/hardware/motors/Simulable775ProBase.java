package frc.examples.hardware.motors;

import edu.wpi.first.wpilibj.SpeedController;

public abstract class Simulable775ProBase<T extends SpeedController> extends SimulableMotor<T> {

    public Simulable775ProBase(int deviceNumber) {
        super(deviceNumber);
        
        // These should be constants
        freeSpeedRPM = 18730;
        freeCurrentAmps = 0.7;
        stallTorqueNM = 0.71;
        stallCurrentAmps = 134;
    }
}
