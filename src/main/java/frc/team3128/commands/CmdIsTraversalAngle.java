package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdIsTraversalAngle extends CommandBase{
    private final NAR_Drivetrain m_drive;

    public CmdIsTraversalAngle() {
        m_drive = NAR_Drivetrain.getInstance();
        // don't require the drivetrain because we are only pulling data from the gyro
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // note that pitch is only used because the gyro is mounted on wrong rn
        return m_drive.getPitch() >= 10 && m_drive.getPitchRate() <= -20;
    }
}