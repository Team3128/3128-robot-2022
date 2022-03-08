package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.ClimberConstants;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Climber;

public class CmdClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double m_distance;
    private boolean isGoingDown;

    // Bottom = 0
    // Top ~~ -7700
    public CmdClimbEncoder(Climber climber, double distance) {
        m_climber = climber;
        m_distance = distance;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        //leftTicks = m_climber.getCurrentTicksLeft() + m_climber.getDesiredTicks(m_distance);
        //if distance is greater than current encoder value, going up
        if (m_distance > m_climber.getCurrentTicksLeft()) {
            m_climber.bothExtend();
            isGoingDown = false;
            Log.info("CmdClimbEncoder", "going up");
        }
        //if distance is less than current encoder value, go down
        else {
            m_climber.bothRetract();
            isGoingDown = true;
            Log.info("CmdClimbEncoder", "going down");
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Log.info("CmdClimbEncoder", "done");
        m_climber.bothStop();
    }

    @Override
    public boolean isFinished() {
        //return (Math.abs(leftTicks - m_climber.getCurrentTicksLeft())) <= Constants.ClimberConstants.CLIMBER_ERROR_RATE;
        if (isGoingDown) {
            return (m_climber.getCurrentTicksLeft() <= m_distance + ClimberConstants.TOLERANCE_TICKS);
        } else {
            return (m_climber.getCurrentTicksLeft() >= m_distance - ClimberConstants.TOLERANCE_TICKS);
        }
    }
}