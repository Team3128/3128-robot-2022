package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Climber.ClimberState;

public class CmdClimbExtend extends CommandBase{
    private final Climber m_climber;

    public CmdClimbExtend(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.leftExtend();
        m_climber.rightExtend();
    }


    @Override
    public void execute() {
        if (m_climber.getLeftState() == ClimberState.TOP) {
            m_climber.leftStop();
        }
        if (m_climber.getRightState() == ClimberState.TOP) {
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
        return (m_climber.getLeftState() == ClimberState.TOP && m_climber.getRightState() == ClimberState.TOP);
    }
}