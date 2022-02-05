package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Climber.ClimberState;

public class CmdClimbRetract extends CommandBase{
    private final Climber m_climber;

    public CmdClimbRetract(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.leftRetract();
        m_climber.rightRetract();
    }


    @Override
    public void execute() {
        if (m_climber.getLeftState() == ClimberState.BOTTOM) {
            m_climber.leftStop();
        }
        if (m_climber.getRightState() == ClimberState.BOTTOM) {
            m_climber.rightStop();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.leftStop();
        m_climber.rightStop();
    }

    @Override
    public boolean isFinished() {
        return (m_climber.getLeftState() == ClimberState.BOTTOM && m_climber.getRightState() == ClimberState.BOTTOM);
    }
}