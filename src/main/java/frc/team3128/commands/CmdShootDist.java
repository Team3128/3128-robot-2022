package frc.team3128.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;
import static frc.team3128.Constants.SimConstants.*;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hood hood;
    private Hopper hopper;

    /**
     * Shoot through calculating the approximate distance to target via the limelight 
     * and initialize+execute the PID loops for the shooter and hood
     * @Requirements Shooter, Hood
     */
    public CmdShootDist() {
        shooter = Shooter.getInstance();
        limelights = LimelightSubsystem.getInstance();
        hood = Hood.getInstance();
        hopper = Hopper.getInstance();

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        limelights.turnShooterLEDOn();
        hopper.runHopper(-0.1);
        shooter.resetPlateauCount();
    }
    
    @Override
    public void execute() {
        double dist = (Robot.isSimulation()) ? SIM_HUB_DISTANCE : limelights.calculateDistance("shooter");
        shooter.beginShoot(shooter.calculateMotorVelocityFromDist(dist));
        hood.startPID(hood.calculateAngleFromDistance(dist));
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        hopper.stopHopper();
        limelights.turnShooterLEDOff();
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
