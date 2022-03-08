package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants.ClimberConstants;

public class CmdClimb extends SequentialCommandGroup{

    public CmdClimb(Climber m_climber){
        addCommands(
            //Climber is manually fully retracted on Mid Bar
            new CmdClimbEncoder(m_climber, -350),

            new WaitCommand(1),
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),
           
            new WaitCommand(1),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            new WaitCommand(0.5),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
            
            new WaitCommand(0.5),

            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            new WaitCommand(1),

            //elev retract
            // new CmdClimbEncoder(m_climber, -350)
            new CmdClimbEncoder(m_climber, 3000) // Aaron number
        );
    }

}