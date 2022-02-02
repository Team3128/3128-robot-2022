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
        m_climber.climberLeftExtend();
        m_climber.climberRightExtend();
    }


    @Override
    public void execute() {
        if (m_climber.getClimberLeftState() == ClimberState.TOP) {
            m_climber.climberLeftStop();
        }
        if (m_climber.getClimberRightState() == ClimberState.TOP) {
            m_climber.climberRightStop();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.climberLeftStop();
        m_climber.climberRightStop();
    }

    @Override
    public boolean isFinished() {
        return (m_climber.getClimberLeftState() == ClimberState.TOP && m_climber.getClimberRightState() == ClimberState.TOP);
    }
}