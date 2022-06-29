package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.team3128.Constants.IntakeConstants.*;
import frc.team3128.subsystems.Intake;

// Extending WaitCommand is the same functionality as .withTimeout
public class CmdExtendIntake extends WaitCommand {
    
    private Intake m_intake;

    /**
     * Extends intake to intaking position with built-in timeout of 0.125 sec
     * @Requirements Intake
     */
    public CmdExtendIntake() {
        super(0.125); 
        m_intake = Intake.getInstance();

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_intake.runIntake(OUTTAKE_MOTOR_POWER);

        m_intake.ejectIntake();
    }

    @Override 
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_intake.stopIntake();
    }

}
