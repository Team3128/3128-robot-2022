package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdIsTraversalAngle extends CommandBase{

    private final NAR_Drivetrain m_drive;
    
    //Pitch is within [-90,+90] degrees.
    private double pitch;

    // use > or <
    private String sign;

    /**
     * Command to calculate when robot is at correct position during its swing to move from high to traversal
     */
    public CmdIsTraversalAngle(String sign,double pitch) {
        m_drive = NAR_Drivetrain.getInstance();
        this.pitch = pitch;
        this.sign = sign;
        // don't require the drivetrain because we are only pulling data from the gyro
    }

    @Override
    public boolean isFinished() {
        if (sign == "<") {
            return m_drive.getPitch() <= pitch;
        }
        else if (sign == ">") {
            return m_drive.getPitch() >= pitch;
        } 
        else {
            return true;
        }
    }
}