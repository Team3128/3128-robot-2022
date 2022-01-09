package frc.team3128.common;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

abstract public class NAR_PIDSubsystem extends PIDSubsystem{
    
    // number of successful checks
    private int plateauCount = 0;
    // threshold value to which plateauCount must reach to be ready
    private int plateauThreshold;

    protected final PIDController m_controller;


    /**
     * Sets default value of plateauThreshold to 1 if the PIDSubsystem doesn't have a threshold value in Constants
     * @param controller PIDController used for the PID loop
     */
    public NAR_PIDSubsystem(PIDController controller) {
        super(controller);
        m_controller = controller;
        plateauThreshold = 1;
    }


    /**
     * Initalizes PIDController and plateauThreshold values 
     * @param controller PIDController used for the PID loop
     * @param plateauThreshold Threshold value (most likely from Constants) to which RPM will reach
     */
    public NAR_PIDSubsystem(PIDController controller, int plateauThreshold) {
        super(controller);
        m_controller = controller;
        this.plateauThreshold = plateauThreshold;
    }


    /**
     * Increment the plateau counter for each iteration that our RPM is at the setpoint. If 
     * the RPM drifts from the setpoint, set plateau count back to zero.
     * @param setpoint The desired setpoint RPM for the PID Loop
     */
    public void useOutput(double setpoint) {
        if (getController().atSetpoint() && (setpoint != 0)) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }
    }

    /**
     * Set plateau counter back to 0 when restarted PID loop
     */
    public void startPID() {
        plateauCount = 0;
    }


    /**
     * @return if the PIDSubsystem has maintained the desired RPM for a certain amount of iterations
     */
    public boolean isReady() {
        return (plateauCount >= plateauThreshold);
    }
}
