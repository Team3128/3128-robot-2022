package frc.team3128.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants;

public class ClimbShort extends CommandBase{
    private final Climber m_climber;

    public ClimbShort(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.resetEncoder();
        m_climber.climberExtend();
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
        return m_climber.getCurrentTicks() == m_climber.getDesiredTick(Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE);

    }
}