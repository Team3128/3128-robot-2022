package frc.team3128.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Intake;


public class StopIntake extends CommandBase {
    private final Intake m_intake;

    public StopIntake(Intake intake){
        m_intake = intake; 
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.stopIntake();
    }
    
    
    @Override
    public boolean isFinished() {
        return false;
    }
}