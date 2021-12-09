package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.*;

public class Mover extends SubsystemBase {
    public static BaseTalon motor;
    public static TalonSRXSimCollection motorSim;

    public Mover() {
        configHardware();
    }

    private void configHardware() {
        if (Robot.isReal()) {
            motor = new TalonFX(0);
        }
        else {
            motor = new TalonSRX(0);
            motorSim = new TalonSRXSimCollection(motor);
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        
    }

}

