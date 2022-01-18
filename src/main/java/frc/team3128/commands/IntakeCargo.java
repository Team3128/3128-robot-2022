package frc.team3128.commands;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCargo extends CommandBase{
    private final Intake m_intake; 
    private final Hopper m_hopper;


    public IntakeCargo(Intake intake, Hopper hopper){
        m_intake = intake;
        m_hopper = hopper;
        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.ejectIntake();
        m_intake.runIntake();
        m_hopper.runHopper();
    }

    public void execute(){
        
    }

    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_intake.retractIntake();
        m_hopper.stopHopper();
    }

    public boolean isFinished(){
        return false;
    }


}
