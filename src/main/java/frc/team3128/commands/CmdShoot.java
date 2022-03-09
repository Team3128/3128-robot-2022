package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Shooter.ShooterState;

public class CmdShoot extends CommandBase {
    private Shooter shooter;
    private ShooterState state;
    private Hood adjustableShooter; 
    
    public CmdShoot(Shooter shooter, Hood adjustableShooter, ShooterState state) {
        this.shooter = shooter;
        this.state = state;
        this.adjustableShooter = adjustableShooter;

        addRequirements(shooter, adjustableShooter);
    }
    
    @Override
    public void initialize() {
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
