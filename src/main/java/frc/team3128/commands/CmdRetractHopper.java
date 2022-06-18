package frc.team3128.commands;
import static frc.team3128.Constants.HopperConstants.*;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CmdRetractHopper extends WaitCommand {
    private Hopper m_hopper;

    /**
     * Runs hopper back to balls against intake and away from shooter flywheel
     * 
     * Intended to allow shooter to ramp up to speed
     */
    public CmdRetractHopper() {
        super(0.5);
        m_hopper = Hopper.getInstance(); 
    }

    @Override
    public void initialize() {
        super.initialize();
        m_hopper.resetEncoder();
        m_hopper.reverseHopper(REVERSE_HOPPER_MOTOR_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_hopper.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || m_hopper.getHopperDistance() <= HOPPER_MAX_REVERSE_DISTANCE;
    }
}