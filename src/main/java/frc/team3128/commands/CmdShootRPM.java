package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Shooter;

public class CmdShootRPM extends CommandBase {
    private Shooter shooter;
    private double rpm;
    
    public CmdShootRPM(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;

        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.beginShoot(rpm);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        Log.info("command shoot", "im cancelling");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
