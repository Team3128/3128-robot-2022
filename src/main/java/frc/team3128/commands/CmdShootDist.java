package frc.team3128.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Robot;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Shooter;
import static frc.team3128.Constants.SimConstants.*;

public class CmdShootDist extends CommandBase {
    private Shooter shooter;
    private LimelightSubsystem limelights;
    private Hood hood;
    
    public CmdShootDist() {
        this.shooter = Shooter.getInstance();
        this.limelights = LimelightSubsystem.getInstance();
        this.hood = Hood.getInstance();

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        limelights.turnShooterLEDOn();
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
        limelights.turnShooterLEDOff();
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
