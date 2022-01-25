package frc.team3128.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants;

public class ClimbSensor extends CommandBase{
    private final Climber m_climber;
    private final BooleanSupplier isShooting;
    private boolean isEjected;

    public ClimbSensor(Climber climber) {
        m_climber = climber;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}