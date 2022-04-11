package frc.team3128.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdArcadeDrive extends CommandBase {

    private final NAR_Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_turn;
    private final DoubleSupplier m_throttle;
    private final BooleanSupplier m_halfSpeed;

    private SlewRateLimiter filter = new SlewRateLimiter(DriveConstants.ARCADE_DRIVE_RATE_LIMIT);

    public CmdArcadeDrive(NAR_Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier turn, DoubleSupplier throttle, BooleanSupplier halfSpeed) {
        m_drivetrain = drivetrain;
        
        m_xSpeed = xSpeed;
        m_turn = turn;
        m_throttle = throttle;
        m_halfSpeed = halfSpeed;

        addRequirements(m_drivetrain);
    }
    
    @Override
    public void execute() {
        double throttle = m_throttle.getAsDouble();

        if (m_halfSpeed.getAsBoolean()) {
            throttle *= 0.5;
        }

        double xSpeed = m_xSpeed.getAsDouble();
        double turn = DriveConstants.ARCADE_DRIVE_TURN_MULT * m_turn.getAsDouble();

        m_drivetrain.arcadeDrive(filter.calculate(xSpeed * throttle), turn);
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
