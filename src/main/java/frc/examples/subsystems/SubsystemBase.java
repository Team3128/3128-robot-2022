package frc.examples.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class SubsystemBase implements Subsystem {
    public SubsystemBase() {
        configActuators();
        configSensors();
    }

    protected abstract void configActuators();
    protected abstract void configSensors();

    public void periodic() {
        periodicUpdate();
    }

    public void simulationPeriodic() {
        periodicUpdate();
    }

    protected abstract void periodicUpdate();
}
