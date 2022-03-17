package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Shooter;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Hood hood;
    
    public CmdShootDist(Shooter shooter, Hood hood, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.hood = hood;

        addRequirements(shooter, hood);
    }
    
    @Override
    public void execute() {
        double dist = limelight.calculateDistToTopTarget(VisionConstants.TARGET_HEIGHT) - 11; // (dist to LL) - (dist from LL to front)
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDistance(dist));
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
