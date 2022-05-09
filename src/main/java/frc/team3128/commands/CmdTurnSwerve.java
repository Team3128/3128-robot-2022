package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.SwerveTrain;

public class CmdTurnSwerve extends CommandBase{
    public SwerveTrain swerve;
    public int degrees;

    public CmdTurnSwerve(SwerveTrain swerve, int degrees){
        this.swerve = swerve;
        this.degrees = degrees;
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean interrupted){
        swerve.drive(0,0,degrees,true);
    }
}
