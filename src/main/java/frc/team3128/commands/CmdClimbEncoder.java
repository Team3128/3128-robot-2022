package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.ClimberConstants.*;
import frc.team3128.subsystems.Climber;

public class CmdClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double distance;
    private boolean isGoingDown;

    public CmdClimbEncoder(double distance) {
        m_climber = Climber.getInstance();
        this.distance = distance;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {

        if (distance > m_climber.getCurrentTicks()) { // want to go higher than current place = go up
            m_climber.bothExtend();
            isGoingDown = false;
        }
        else { // want to go lower than current place = go down
            m_climber.bothRetract();
            isGoingDown = true;
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
        if (isGoingDown) {
            return (m_climber.getCurrentTicks() <= distance + TOLERANCE_TICKS);
        } else {
            return (m_climber.getCurrentTicks() >= distance - TOLERANCE_TICKS);
        }
    }
}