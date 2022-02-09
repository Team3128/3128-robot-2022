package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants;

public class CmdClimb extends SequentialCommandGroup{

    public CmdClimb(Climber m_climber){
        addCommands(
            //Climber is manually fully retracted on Mid Bar
            
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE),
           
            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            //elev extend
            new CmdClimbExtend(m_climber),
            
            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            //elev retract
            new CmdClimbRetract(m_climber),

            //repeat above commands:
            
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE),
           
            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            //elev extend
            new CmdClimbExtend(m_climber),
            
            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            //elev retract
            new CmdClimbRetract(m_climber)

        );
    }

}