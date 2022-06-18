package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.ConstantsInt;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Shooter;

public class CmdShootRPM extends CommandBase {
    private Shooter shooter;
    private Hopper hopper;
    private double rpm;

    /**
     * Shoot through initializing+executing the PID loop for the shooter with a parameter RPM
     * @param rpm Target RPM for the shooter 
     * @Requirements Shooter
     */
    public CmdShootRPM(double rpm) {
        shooter = Shooter.getInstance();
        hopper = Hopper.getInstance();
        this.rpm = rpm;

        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        // rpm = ConstantsInt.ShooterConstants.SET_RPM;
        shooter.resetPlateauCount();
        shooter.beginShoot(rpm);
        hopper.runHopper(-0.1);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        hopper.stopHopper();
        Log.info("CmdShootRPM", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
