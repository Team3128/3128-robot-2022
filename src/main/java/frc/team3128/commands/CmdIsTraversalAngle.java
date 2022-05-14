package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdIsTraversalAngle extends CommandBase{
    private final NAR_Drivetrain m_gyro;

    public CmdIsTraversalAngle(NAR_Drivetrain gyro) {
        m_gyro = gyro;

        addRequirements(gyro);
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
        return m_gyro.getPitch() >= 10 && m_gyro.getPitchRate() <= -20;
    }
}