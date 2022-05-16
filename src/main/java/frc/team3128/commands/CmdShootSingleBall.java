package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;

public class CmdShootSingleBall extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hood hood;

    private boolean prevIsReady = false;
    private boolean currIsReady;
    
    public CmdShootSingleBall(Shooter shooter, Hood hood, LimelightSubsystem limelights) {
        this.shooter = shooter;
        this.limelights = limelights;
        this.hood = hood;

        addRequirements(shooter, hood);
    }
    
    @Override
    public void execute() {
        double dist = limelights.calculateDistance("shooter");
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDistance(dist));

        prevIsReady = currIsReady;
        currIsReady = shooter.isReady();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        limelights.turnShooterLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return prevIsReady && !currIsReady;
    }
}
