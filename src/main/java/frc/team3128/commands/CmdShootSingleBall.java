package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;

public class CmdShootSingleBall extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hopper hopper;
    private Hood hood;

    private boolean prevIsReady = false;
    private boolean currIsReady;
    
    /**
     * Shoot a single ball when two in robot
     * 
     * Uses limelight approximated distance for the shooter and hood PID loops
     * @Requirements Shooter, hood
     */
    public CmdShootSingleBall() {
        shooter = Shooter.getInstance();
        limelights = LimelightSubsystem.getInstance();
        hood = Hood.getInstance();
        hopper = Hopper.getInstance();

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        hopper.runHopper(-0.1);
    }
    
    @Override
    public void execute() {
        double dist = limelights.calculateShooterDistance();
        shooter.beginShoot(shooter.calculateRPMFromDist(dist));
        hood.startPID(hood.calculateAngleFromDist(dist));

        prevIsReady = currIsReady;
        currIsReady = shooter.isReady();
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        hopper.stopHopper();
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
