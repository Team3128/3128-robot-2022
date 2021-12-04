package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Constants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_turn;
    private final DoubleSupplier m_throttle;
    
    public ArcadeDrive(NAR_Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier turn, DoubleSupplier throttle) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        m_xSpeed = xSpeed;
        m_turn = turn;
        m_throttle = throttle;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        double throttle = (1 - m_throttle.getAsDouble())/2; // Map throttle from [-1, 1] to [0, 1]

        m_drivetrain.arcadeDrive(m_xSpeed.getAsDouble() * throttle, Constants.DriveConstants.ARCADE_DRIVE_TURN_MULT * m_turn.getAsDouble() * throttle);
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
