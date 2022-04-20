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

    private double pastSpeed = 0;
    private double currSpeed;
    
    public CmdShootSingleBall(Shooter shooter, Hood hood, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.hood = hood;

        addRequirements(shooter, hood);
    }
    
    @Override
    public void execute() {
        double dist = limelight.calculateDistToTopTarget(VisionConstants.TARGET_HEIGHT);
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDistance(dist));

        pastSpeed = currSpeed;
        currSpeed = shooter.getMeasurement();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        limelight.turnLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return pastSpeed - currSpeed > 250;
    }
}
