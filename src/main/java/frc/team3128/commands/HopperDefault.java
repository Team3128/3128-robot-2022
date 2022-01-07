package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;

public class HopperDefault extends CommandBase {
    private final Hopper m_hopper;
    private final BooleanSupplier shooterIsReady;
    private final BooleanSupplier sidekickIsReady;


    public HopperDefault(Hopper hopper, BooleanSupplier shooterIsReady, BooleanSupplier sidekickIsReady) {
        m_hopper = hopper;
        this.shooterIsReady = shooterIsReady;
        this.sidekickIsReady = sidekickIsReady;

        addRequirements(m_hopper);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        boolean isTop = m_hopper.getTop();
        boolean isBottom = m_hopper.getBottom();
        boolean isShooting = this.shooterIsReady.getAsBoolean() && this.sidekickIsReady.getAsBoolean();

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
