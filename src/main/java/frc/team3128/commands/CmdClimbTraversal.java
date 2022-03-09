package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants.ClimberConstants;

public class CmdClimbTraversal extends SequentialCommandGroup{

    public CmdClimbTraversal(Climber m_climber) {
        addCommands(
            
            // new CmdExtendIntake(m_intake),

            //Climber is manually fully retracted on Mid Bar
            new InstantCommand(() -> m_climber.retractPiston()),

            new CmdClimbEncoder(m_climber, -350),

            new WaitCommand(0.5),
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),

            new WaitCommand(0.5),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            new WaitCommand(0.25),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
            
            new WaitCommand(0.25),

            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            new WaitCommand(0.5),

            //elev retract
            new CmdClimbEncoder(m_climber, -350),

            //Climber is manually fully retracted on High Bar

            //wait between sequences
            new WaitCommand(1.75),
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),

            // new CmdExtendIntake(m_intake),

            new WaitCommand(0.5),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            new WaitCommand(0.25),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
            
            new WaitCommand(0.25),

            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            new WaitCommand(0.5),

            //elev retract
            new CmdClimbEncoder(m_climber, 3000) // Aaron number
        );
    }

}