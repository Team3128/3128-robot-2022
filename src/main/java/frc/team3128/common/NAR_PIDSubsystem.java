package frc.team3128.common;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.subsystems.*;
import edu.wpi.first.wpilibj.controller.PIDController;

abstract public class NAR_PIDSubsystem extends PIDSubsystem{
    
    private int plateauCount;
    private int plateauThreshold;

    protected final PIDController m_controller;

    public NAR_PIDSubsystem(PIDController controller) {
        super(controller);
        m_controller = controller;
        plateauCount = 0;
        plateauThreshold = 1;
    }

    public NAR_PIDSubsystem(PIDController controller, int plateauThreshold) {
        super(controller);
        m_controller = controller;
        this.plateauThreshold = plateauThreshold;
    }

    public void useOutput(double setpoint) {
        if (getController().atSetpoint() && (setpoint != 0)) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }
    }

    public void startPID() {
        plateauCount = 0;
    }


    public boolean isReady() {
        return (plateauCount >= plateauThreshold);
    }
}
