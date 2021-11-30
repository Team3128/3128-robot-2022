package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;

public class HopperDefault extends CommandBase {
    private final Hopper m_hopper;
    private final BooleanSupplier isShooting;

    public HopperDefault(Hopper hopper, BooleanSupplier isShooting) {
        m_hopper = hopper;
        this.isShooting = isShooting;

        addRequirements(m_hopper);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        boolean isTop = m_hopper.getTop();
        boolean isBottom = m_hopper.getBottom();
        boolean isShooting = this.isShooting.getAsBoolean();

        if (isShooting) {
            m_hopper.runHopper(1);
        }
        else {
            if (isBottom && !isTop) {
                m_hopper.runHopper(1);
            }
            else {
                m_hopper.stopHopper();
            }
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
