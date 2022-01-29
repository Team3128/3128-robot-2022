package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Climber.ClimberState;

public class ClimbExtend extends CommandBase{
    private final Climber m_climber;

    public ClimbExtend(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.climberExtend();
    }


    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.climberStop();
    }

    @Override
    public boolean isFinished() {
        return m_climber.getClimberState() == ClimberState.TOP ;
    }
}