package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
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
        double throttle = (-m_throttle.getAsDouble() + 1) / 2;
        if(throttle < 0.3)
            throttle = 0.3;
        if (throttle > 0.8)
            throttle = 1;


        double xSpeed = -m_xSpeed.getAsDouble(); //invert xSpeed

        double turn = Constants.DriveConstants.ARCADE_DRIVE_TURN_MULT * m_turn.getAsDouble();
        if (Math.abs(turn) < Constants.DriveConstants.ARCADE_DRIVE_TURN_DEADBAND)
            turn = 0;

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
