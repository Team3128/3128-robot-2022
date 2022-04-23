package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Shooter;

public class CmdShootSingleBall extends CommandBase {
    private Shooter shooter;
    private Limelight limelight;
    private Hood hood;

    private boolean prevIsReady = false;
    private boolean currIsReady;
    
    public CmdShootSingleBall(Shooter shooter, Hood hood, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.hood = hood;

        addRequirements(shooter, hood);
    }
    
    @Override
    public void execute() {
        double dist = limelight.calculateDistToTopTarget(VisionConstants.TARGET_HEIGHT) + 5;
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDistance(dist));

        prevIsReady = currIsReady;
        currIsReady = shooter.isReady();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        limelight.turnLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return prevIsReady && !currIsReady;
    }
}
