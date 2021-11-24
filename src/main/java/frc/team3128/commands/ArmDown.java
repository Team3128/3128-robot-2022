package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Intake;

public class ArmDown extends CommandBase{
    private final Intake m_intake;
    
    public ArmDown(Intake intake){
        m_intake = intake; 
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        m_intake.moveArmDown();
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
