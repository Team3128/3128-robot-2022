package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class TestBenchPiston extends SubsystemBase {
    private DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    public TestBenchPiston(){}  

    public void eject(){
        m_solenoid.set(kForward);
    }

    public void retract(){
        m_solenoid.set(kReverse);
    }

    public void off() {
        m_solenoid.set(kOff); 
    }
}
