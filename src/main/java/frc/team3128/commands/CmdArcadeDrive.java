package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdArcadeDrive extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_turn;
    private final DoubleSupplier m_throttle;


    public CmdArcadeDrive(NAR_Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier turn, DoubleSupplier throttle) {
        m_drivetrain = drivetrain;
        
        m_xSpeed = xSpeed;
        m_turn = turn;
        m_throttle = throttle;

        addRequirements(m_drivetrain);
    }
    
    @Override
    public void execute() {
        double throttle = m_throttle.getAsDouble();

        double xSpeed = m_xSpeed.getAsDouble();
        double turn = DriveConstants.ARCADE_DRIVE_TURN_MULT * m_turn.getAsDouble();

        m_drivetrain.arcadeDrive(xSpeed * throttle, turn * throttle);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
