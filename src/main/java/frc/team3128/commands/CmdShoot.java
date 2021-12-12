package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Sidekick;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.subsystems.Sidekick.SidekickState;

public class CmdShoot extends CommandBase {
    private final Shooter m_shooter;
    private final Sidekick m_sidekick;
    private final ShooterState m_state;
    
    public CmdShoot(Shooter shooter, Sidekick sidekick, ShooterState state) {
        m_shooter = shooter;
        m_sidekick = sidekick;
        m_state = state;

        addRequirements(m_shooter, m_sidekick);
    }
    
    @Override
    public void execute() {
        m_shooter.beginShoot(m_state);
        m_sidekick.beginShoot(SidekickState.DEFAULT);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShoot();
        m_sidekick.stopShoot();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
