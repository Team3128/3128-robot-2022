package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;

public class StopShoot extends CommandBase {
    private final Shooter m_shooter;
    
    public StopShoot(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }
    
    @Override
    public void initialize() {
        m_shooter.stopShoot();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
