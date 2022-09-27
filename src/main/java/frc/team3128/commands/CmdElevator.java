package frc.team3128.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class CmdElevator extends PIDSubsystem{

    private PIDController m_controller;
    
    public CmdElevator(PIDController controller) {
        super(controller); //add num later on
        m_controller = controller;
        
    }

    // important ID elements
    public static final double Bottom = 1.00;
    public static final double Stow = 1.00;
    public static final double Table_Height = 1.00;

    private AnalogPotentiometer pot;
    private PWMTalonSRX motor;

    // @Override
    // public void initialize(){
        
    // }
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return pot.get();
    }
    
}
