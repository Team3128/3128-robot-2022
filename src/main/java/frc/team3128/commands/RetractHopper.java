package frc.team3128.commands;
import frc.team3128.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractHopper extends CommandBase{
    private final Hopper m_hopper;

    public RetractHopper(Hopper hopper){
        m_hopper = hopper;
        addRequirements(m_hopper); 
    }

    @Override
    public void initialize() {
        m_hopper.reverseHopper();
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