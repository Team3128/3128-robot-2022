package frc.team3128.common;

import edu.wpi.first.wpilibj.RobotBase;

public abstract class Simulable {
    protected boolean isReal;

    public void construct(){
        constructReal();
        isReal = RobotBase.isSimulation();
        if(RobotBase.isSimulation()){
            constructFake();
        }
    }

    public abstract void constructReal();
    public abstract void constructFake();
    public abstract void updateSimulation(double timeStep);
}