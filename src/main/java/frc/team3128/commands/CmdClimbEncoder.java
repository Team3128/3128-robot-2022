package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.ClimberConstants.*;
import frc.team3128.subsystems.Climber;

public class CmdClimbEncoder extends CommandBase{
    private final Climber m_climber;
    private final double distance;
    private boolean isGoingDown;
    private final double power;

    /**
     * Moves climber arms to stated distance
     * @param distance distance to move arms to (encoder ticks)
     * @Requirements Climber
     */
    public CmdClimbEncoder(double distance) {
        m_climber = Climber.getInstance();
        this.distance = distance;
        this.power = CLIMBER_POWER;
        addRequirements(m_climber);
    }

    public CmdClimbEncoder(double distance, double power) {
        m_climber = Climber.getInstance();
        this.distance = distance;
        this.power = power;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        
        // want to go higher than current place = go up
        if (distance > m_climber.getCurrentTicks()) { 
            m_climber.bothExtend(power);
            isGoingDown = false;
        }
        else { // want to go lower than current place = go down
            m_climber.bothRetract(power);
            isGoingDown = true;
        }
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