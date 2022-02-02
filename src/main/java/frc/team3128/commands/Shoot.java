package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;

public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final ShooterState state;
    
    public Shoot(Shooter shooter, ShooterState state) {
        this.shooter = shooter;
        this.state = state;

        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.beginShoot(state);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
