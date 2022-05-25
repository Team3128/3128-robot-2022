package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;

public class CmdShootSingleBall extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hood hood;

    private boolean prevIsReady = false;
    private boolean currIsReady;
    
    public CmdShootSingleBall() {
        this.shooter = Shooter.getInstance();
        this.limelights = LimelightSubsystem.getInstance();
        this.hood = Hood.getInstance();

        addRequirements(shooter, hood);
    }
    
    @Override
    public void execute() {
        double dist = limelights.calculateShooterDistance();
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDist(dist));

        prevIsReady = currIsReady;
        currIsReady = shooter.isReady();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        limelights.turnShooterLEDOff();
        Log.info("CmdShootSingleBall", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        // this could be adjusted with more testing
        // idea is to see if there is a RPM drop 
        return prevIsReady && !currIsReady;
    }
}
