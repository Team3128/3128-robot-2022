package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Intake;

public class CmdExtendIntake extends WaitCommand {
    
    private Intake m_intake;

    public CmdExtendIntake(Intake intake) {
        super(0.125);
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_intake.runIntakeBack();

        m_intake.ejectIntake();
    }

    @Override 
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_intake.stopIntake();
    }

}
