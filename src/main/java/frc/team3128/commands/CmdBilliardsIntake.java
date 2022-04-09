package frc.team3128.commands;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdBilliardsIntake extends CommandBase{
    private Intake m_intake; 
    private Hopper m_hopper;
    private double power;


    public CmdBilliardsIntake(Intake intake, Hopper hopper, double power){
        m_intake = intake;
        m_hopper = hopper;
        this.power = power;

        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.runIntakeBack(power);
        m_hopper.reverseHopper(power);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_intake.retractIntake();
    }

    public boolean isFinished(){
        return false;
    }
}
