package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hood hood;

    /**
     * Shoot through calculating the approximate distance to target via the limelight 
     * and initialize+execute the PID loops for the shooter and hood
     * @Requirements Shooter, Hood
     */
    public CmdShootDist() {
        shooter = Shooter.getInstance();
        limelights = LimelightSubsystem.getInstance();
        hood = Hood.getInstance();

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        limelights.turnShooterLEDOn();
        shooter.resetPlateauCount();
    }
    
    @Override
    public void execute() {
        double dist = limelights.calculateShooterDistance();
        shooter.beginShoot(shooter.calculateRPMFromDist(dist));
        hood.startPID(hood.calculateAngleFromDist(dist));
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        limelights.turnShooterLEDOff();
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
