package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.ConstantsInt;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Shooter;

public class CmdShootRPM extends CommandBase {
    private Shooter shooter;
    private double rpm;
    
    public CmdShootRPM(double rpm) {
        this.shooter = Shooter.getInstance();
        this.rpm = rpm;

        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        // rpm = ConstantsInt.ShooterConstants.SET_RPM;
        shooter.beginShoot(rpm);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        Log.info("CmdShootRPM", "Cancelling shooting");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
