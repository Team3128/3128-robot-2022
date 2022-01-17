package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Hopper;

public class HopperDefault extends CommandBase {
    private final Hopper m_hopper;
    private final BooleanSupplier isShooting;
    private boolean isEjected;

    public HopperDefault(Hopper hopper, BooleanSupplier isShooting) {
        m_hopper = hopper;
        this.isShooting = isShooting;

        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public void execute() {
        isEjected = m_hopper.getEjected();

        if (isShooting.getAsBoolean()) {
            if (isEjected) m_hopper.retractPistonGate();
            m_hopper.runHopper();
        } else {
            if (!isEjected) m_hopper.ejectPistonGate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.turnPistonOff();
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}