package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Shooter;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    
    public CmdShootDist(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(
            limelight.calculateDistToTopTarget(VisionConstants.TARGET_HEIGHT)));
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        Log.info("command shoot", "im cancelling");
        limelight.turnLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
