package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Robot;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import static frc.team3128.Constants.SimConstants.*;
import static frc.team3128.Constants.DriveConstants.*;

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
        double dist = limelights.calculateShooterDistance();
        if (Robot.isSimulation()) {
             
            Pose2d pose = NAR_Drivetrain.getInstance().getPose().relativeTo(HUB_POS);

            dist = Units.metersToInches(Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY())) - HUB_RADIUS; // inches

            SmartDashboard.putNumber("Distance from hub (meters)", Units.inchesToMeters(dist));
        }
        shooter.beginShoot(shooter.calculateRPMFromDist(dist));
        hood.startPID(hood.calculateAngleFromDist(dist));
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        hopper.stopHopper();
        limelights.turnShooterLEDOff();
        Log.info("CmdShootDist", "Cancelling shooting");

        if (Robot.isSimulation()) {
            hopper.resetBallCount();
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
