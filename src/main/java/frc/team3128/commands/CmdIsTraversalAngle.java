package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdIsTraversalAngle extends CommandBase{
    private final NAR_Drivetrain m_drive;

    /**
     * Command to calculate when robot is at correct position during its swing to move from high to traversal
     */
    public CmdIsTraversalAngle() {
        m_drive = NAR_Drivetrain.getInstance();
        // don't require the drivetrain because we are only pulling data from the gyro
    }

    @Override
    public boolean isFinished() {
        // pitch is used because gyro is mounted on wrong currently 
        return m_drive.getPitch() <= 5 && m_drive.getPitchRate() >= 0.1;
    }
}