package frc.team3128.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.subsystems.Hopper;

public class CmdHopperShooting extends CommandBase {

    private Hopper m_hopper;
    private BooleanSupplier isShooting;
    private double power;

    public CmdHopperShooting(Hopper hopper, BooleanSupplier isShooting) {
        this(hopper, isShooting, Constants.HopperConstants.HOPPER_MOTOR_POWER);
    }
    public CmdHopperShooting(Hopper hopper, BooleanSupplier isShooting, double power) {
        m_hopper = hopper;
        this.isShooting = isShooting;
        this.power = power;

        addRequirements(m_hopper);
    }

    @Override
    public void execute() {
        // if shooting, retract gate if ejected and run the hopper
        if (isShooting.getAsBoolean()) { 
            // m_hopper.runHopper();
            m_hopper.runHopper(power);
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