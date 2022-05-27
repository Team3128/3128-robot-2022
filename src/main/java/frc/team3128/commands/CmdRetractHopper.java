package frc.team3128.commands;
import static frc.team3128.Constants.HopperConstants.*;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CmdRetractHopper extends CommandBase {
    private Hopper m_hopper;

    public CmdRetractHopper() {
        m_hopper = Hopper.getInstance();
        // addRequirements(m_hopper); 
    }

    @Override
    public void initialize() {
        m_hopper.resetEncoder();
        m_hopper.reverseHopper(REVERSE_HOPPER_MOTOR_POWER);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return m_hopper.getHopperDistance() <= HOPPER_MAX_REVERSE_DISTANCE;
    }
}