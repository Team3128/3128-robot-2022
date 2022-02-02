package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.subsystems.Climber;

public class ClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double m_distance;
    private double leftTicks, rightTicks;

    public ClimbEncoder(Climber climber, double distance) {
        m_climber = climber;
        m_distance = distance;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        leftTicks = m_climber.getCurrentTicksLeft() + m_climber.getDesiredTicks(m_distance);
        rightTicks = m_climber.getCurrentTicksRight() + m_climber.getDesiredTicks(m_distance);
        if (m_distance > 0) {
            m_climber.climberLeftExtend();
            m_climber.climberRightExtend();
        }
        else if (m_distance < 0) {
            m_climber.climberLeftRetract();
            m_climber.climberRightRetract();
        }
    }

    @Override
    public void execute() {
        if (Math.abs(leftTicks - m_climber.getCurrentTicksLeft()) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE) {
            m_climber.climberLeftStop();
        }
        if (Math.abs(rightTicks - m_climber.getCurrentTicksRight()) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE) {
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
        return (Math.abs(leftTicks - m_climber.getCurrentTicksLeft()) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE) && (Math.abs(rightTicks - m_climber.getCurrentTicksRight()) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE);

    }
}