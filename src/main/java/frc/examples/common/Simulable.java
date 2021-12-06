package frc.examples.common;

import edu.wpi.first.wpilibj.RobotBase;

public abstract class Simulable {
    protected boolean isReal;

    public Simulable() {
        isReal = RobotBase.isReal();
        if(isReal) {
            constructReal();
        } else {
            constructFake();
        }
    }

    public abstract void constructReal();
    public abstract void constructFake();
    public abstract void updateSimulation(double timeStep);
}