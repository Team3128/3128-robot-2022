package frc.team3128.commands;
import frc.team3128.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase{
    private final Intake m_intake; 


    public RunIntake(Intake intake){
        m_intake = intake;
        addRequirements(m_intake);


    }
    @Override
    public void initialize() {
        m_intake.ejectIntake();
        m_intake.runIntake();
    }

    public void execute(){
        
    }

    public void end(boolean interrupted){
        m_intake.stopIntake();
        m_intake.retractIntake();

    }

    public boolean isFinished(){
        return false;
    }


}
