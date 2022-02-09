package frc.team3128.commands;
import frc.team3128.subsystems.Hopper;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdRetractHopperShooting extends CommandBase {
    private Hopper m_hopper;
    private BooleanSupplier isShooting;

    public CmdRetractHopperShooting(Hopper hopper, BooleanSupplier isShooting) {
        m_hopper = hopper;
        this.isShooting = isShooting;
        addRequirements(m_hopper); 
    }

    @Override
    public void initialize() {
        // m_hopper.resetEncoder();
        m_hopper.reverseHopper();
    }

    @Override
    public void execute() {
        if (isShooting.getAsBoolean()) { 
            m_hopper.runHopper();
        } else {
            m_hopper.stopHopper();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return m_hopper.isReversed();
    }
}
