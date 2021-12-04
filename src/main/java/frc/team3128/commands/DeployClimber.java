package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Constants;

public class DeployClimber extends CommandBase {
    private final Climber m_climber; 
    
    public DeployClimber(Climber climber) {
        m_climber = climber; 
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_climber.setMotors(Constants.ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    @Override
    public void end(boolean interrupted) {

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
