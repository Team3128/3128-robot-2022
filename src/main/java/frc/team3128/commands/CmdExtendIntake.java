package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;

public class CmdExtendIntake extends WaitCommand {
    
    private Intake m_intake;

    public CmdExtendIntake(Intake intake) {
        super(0.1);
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (m_intake.getSolenoid().equals("kReverse")) {
            m_intake.runIntakeBack();
        }
        if (m_intake.getSolenoid().equals("kForward")) {
            m_intake.runIntake();
        }

        m_intake.ejectIntake();
    }

    @Override 
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

}
