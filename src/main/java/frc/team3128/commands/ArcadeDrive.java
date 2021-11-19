package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.AN_Drivetrain;

public class ArcadeDrive extends CommandBase {
    private final AN_Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_turn;
    
    public ArcadeDrive(AN_Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier turn) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);

        m_xSpeed = xSpeed;
        m_turn = turn;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(m_xSpeed.getAsDouble(), m_turn.getAsDouble());
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
