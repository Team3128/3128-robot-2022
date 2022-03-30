package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants.ClimberConstants;

public class CmdClimbTraversalOG extends SequentialCommandGroup{

    public CmdClimbTraversalOG(Climber m_climber) {
        addCommands(
            
            // new CmdExtendIntake(m_intake),

            //Climber is manually fully retracted on Mid Bar
            new InstantCommand(() -> m_climber.retractPiston()),

            new CmdClimbEncoder(m_climber, -10000),

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
            new CmdClimbEncoder(m_climber, -10000),

            //Climber is manually fully retracted on High Bar
            
            new WaitCommand(0.5), // 1 weird timing, 

            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION)
            
            // new WaitCommand(0.25),

            // new InstantCommand(() -> m_climber.retractPiston()),

            // new WaitCommand(0.25),

            // new CmdClimbEncoder(m_climber, 1000)

        );
    }

}