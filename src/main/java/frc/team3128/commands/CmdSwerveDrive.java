package frc.team3128.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.subsystems.SwerveTrain;

public class CmdSwerveDrive extends CommandBase{
    private final SwerveTrain m_Swerve;
    private final Supplier<Double> m_xSpd, m_ySpd, m_turningSpd;
    private final boolean m_fieldOriented;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public CmdSwerveDrive(SwerveTrain Swerve, Supplier<Double> xSpd, 
            Supplier<Double> ySpd, Supplier<Double> turningSpeed, 
            boolean fieldOriented){
        m_Swerve = Swerve;
        m_xSpd = xSpd;
        m_ySpd = ySpd;
        m_turningSpd = turningSpeed;
        m_fieldOriented = fieldOriented;
        //Will need a some point probably
        xLimiter = new SlewRateLimiter(1);
        yLimiter = new SlewRateLimiter(1);
        turningLimiter = new SlewRateLimiter(1);

        addRequirements(m_Swerve);
    }
    
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        double xSpeed = m_xSpd.get();
        double ySpeed = m_ySpd.get();
        xSpeed = Math.abs(m_xSpd.get()) > SwerveConstants.kDeadband ? xSpeed:0;
        ySpeed = Math.abs(m_ySpd.get()) > SwerveConstants.kDeadband ? ySpeed:0;
        xSpeed = xLimiter.calculate(m_xSpd.get()) * SwerveConstants.kMaxSpeed;
        ySpeed = yLimiter.calculate(m_ySpd.get()) * SwerveConstants.kMaxSpeed;
        double turnSpeed = turningLimiter.calculate(m_turningSpd.get()) * SwerveConstants.kMaxSpeed;
        m_Swerve.drive(xSpeed,ySpeed,turnSpeed,m_fieldOriented);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
