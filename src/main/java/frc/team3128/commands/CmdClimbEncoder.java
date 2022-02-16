package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.subsystems.Climber;

public class CmdClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double m_distance;
    private double leftTicks;

    public CmdClimbEncoder(Climber climber, double distance) {
        m_climber = climber;
        m_distance = distance;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        leftTicks = m_climber.getCurrentTicksLeft() + m_climber.getDesiredTicks(m_distance);
        if (m_distance > 0) {
            m_climber.bothExtend();
        }
        else if (m_distance < 0) {
            m_climber.bothRetract();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.bothStop();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(leftTicks - m_climber.getCurrentTicksLeft())) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE;
    }
}