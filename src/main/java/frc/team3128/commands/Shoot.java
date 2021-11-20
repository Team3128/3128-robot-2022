package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;

public class Shoot extends CommandBase {
    private final Shooter m_shooter;
    private final ShooterState m_state;
    
    public Shoot(Shooter shooter, ShooterState state) {
        m_shooter = shooter;
        m_state = state;

        addRequirements(m_shooter);
    }
    
    @Override
    public void execute() {
        m_shooter.beginShoot(m_state);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShoot();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
