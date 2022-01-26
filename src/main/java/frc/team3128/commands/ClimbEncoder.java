package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.subsystems.Climber;

public class ClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double m_distance;
    private double ticks;

    public ClimbEncoder(Climber climber, double distance) {
        m_climber = climber;
        m_distance = distance;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        ticks = m_climber.getCurrentTicks() + m_climber.getDesiredTicks(m_distance);
        if (m_distance > 0) {
            m_climber.climberExtend();
        }
        else if (m_distance < 0) {
            m_climber.climberRetract();
        }
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
        return Math.abs(ticks - m_climber.getCurrentTicks()) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE;

    }
}