package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.AdjustableShooter;
import frc.team3128.subsystems.Shooter.ShooterState;

public class CmdShoot extends CommandBase {
    private Shooter shooter;
    private ShooterState state;
    private AdjustableShooter adjustableShooter; 
    
    public CmdShoot(Shooter shooter, AdjustableShooter adjustableShooter, ShooterState state) {
        this.shooter = shooter;
        this.state = state;
        this.adjustableShooter = adjustableShooter;

        addRequirements(shooter, adjustableShooter);
    }
    
    @Override
    public void initialize() {
        shooter.beginShoot(state);
        adjustableShooter.hoodStop(); 
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        adjustableShooter.hoodStop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
