package frc.team3128.common;

import edu.wpi.first.wpilibj.RobotBase;

public abstract class Simulable {
    protected boolean isReal;

    // No constructor is problematic but solves the device-number problem higher up
    public void construct(){
        // We will always need the real code for any simulated class
        constructReal();
        isReal = RobotBase.isSimulation();
        if(isReal){
            constructFake();
        }
    }

    public abstract void constructReal();
    public abstract void constructFake();
    public abstract void updateSimulation(double timeStep);
}