package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Climber.ClimberState;

public class ClimbRetract extends CommandBase{
    private final Climber m_climber;

    public ClimbRetract(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.climberLeftRetract();
        m_climber.climberRightRetract();
    }


    @Override
    public void execute() {
        if (m_climber.getClimberLeftState() == ClimberState.BOTTOM) {
            m_climber.climberLeftStop();
        }
        if (m_climber.getClimberRightState() == ClimberState.BOTTOM) {
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
        return (m_climber.getClimberLeftState() == ClimberState.BOTTOM && m_climber.getClimberRightState() == ClimberState.BOTTOM);
    }
}