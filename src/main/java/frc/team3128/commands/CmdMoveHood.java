package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.HoodConstants;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Hood;

public class CmdMoveHood extends CommandBase {

    private Hood m_hood;
    private double m_targetAngle;

    public CmdMoveHood (Hood hood, double setpointAngle) {
        m_hood = hood;
        m_targetAngle = setpointAngle;

        if (setpointAngle > HoodConstants.MAX_ANGLE || setpointAngle < HoodConstants.MIN_ANGLE) {
            Log.recoverable("CmdMoveHood", "Setpoint angle " + setpointAngle + "out of range"); 
        }

        addRequirements(m_hood);
    }

    @Override
    public void execute() {
        if (m_hood.getAngle() < m_targetAngle) {
            m_hood.setSpeed(HoodConstants.HOOD_SPEED);
        } else if (m_hood.getAngle() > m_targetAngle) {
            m_hood.setSpeed(-HoodConstants.HOOD_SPEED);
        } else {
            m_hood.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(m_hood.getAngle() - m_targetAngle) < HoodConstants.kTolerance;
    }

}
