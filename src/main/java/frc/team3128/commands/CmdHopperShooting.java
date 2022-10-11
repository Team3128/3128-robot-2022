package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Shooter;

public class CmdHopperShooting extends CommandBase {

    private Hopper m_hopper;
    private BooleanSupplier isReady;

    public CmdHopperShooting() {
        m_hopper = Hopper.getInstance();
        isReady = Shooter.getInstance()::isReady;

        addRequirements(m_hopper);
    }

    @Override
    public void execute() {
        
        if (isReady.getAsBoolean()) { 
            m_hopper.runHopper();
        } else {
            m_hopper.runHopper(-0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}