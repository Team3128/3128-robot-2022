package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Intake;

public class CmdExtendIntake extends CommandBase {
    
    private Intake m_intake;

    public CmdExtendIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.ejectIntake();
    }

    @Override
    public void execute() {
        if (m_intake.getSolenoid().equals("kForward")) {
            
        }
    }

    @Override
    public boolean isFinished() {
        
    }

    @Override 
    public void end(boolean interrupted) {

    }

    
}
