package frc.team3128.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Constants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_turn;
    private final DoubleSupplier m_throttle;
    private final BooleanSupplier m_driveInv;

    public ArcadeDrive(NAR_Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier turn, DoubleSupplier throttle, BooleanSupplier driveInversed) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        m_xSpeed = xSpeed;
        m_turn = turn;
        m_throttle = throttle;
        m_driveInv = driveInversed;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {

        double throttle = (-m_throttle.getAsDouble() + 1) / 2;

        if(throttle < 0.3)
            throttle = 0.3;
        if (throttle > 0.8)
            throttle = 1;

        double inv = (m_driveInv.getAsBoolean() ? -1 : 1); // -1 if driveInv is true, 1 if false

        m_drivetrain.arcadeDrive(inv * m_xSpeed.getAsDouble() * throttle, Constants.DriveConstants.ARCADE_DRIVE_TURN_MULT * m_turn.getAsDouble() * throttle);
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
