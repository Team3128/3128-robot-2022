package frc.team3128.commands;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdIntakeCargo extends CommandBase{
    private Intake m_intake; 
    private Hopper m_hopper;

    /**
     * Run intake and hopper forward
     * @Requirements Intake, Hopper
     */
    public CmdIntakeCargo(){
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();

        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.runIntake();
        m_hopper.runHopper();
    }

    @Override
    public void execute() {
        // nothing in execute because runIntake() and runHopper() just set speeds - nothing is updated in command
    }

    @Override
    public void end(boolean interrupted){
        m_intake.retractIntake();
        m_intake.stopIntake();
        m_hopper.stopHopper();
    }

    public boolean isFinished(){
        return false;
    }
}
