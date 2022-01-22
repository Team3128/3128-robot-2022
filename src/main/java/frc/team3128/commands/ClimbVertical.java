package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;

public class ClimbVertical extends CommandBase{
    private final Climber m_climber;
    private final double m_distance; 

    public ClimbVertical(Climber climber, double distance) {
        m_climber = climber;
        m_distance = distance;

        addRequirements();
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {  // scuffed logic; may not work . TODO: ask nathan duggal 
        double ticks = m_climber.getDesiredTicks(m_distance) - m_climber.getCurrentTicks();
        m_climber.setTicks(ticks);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}