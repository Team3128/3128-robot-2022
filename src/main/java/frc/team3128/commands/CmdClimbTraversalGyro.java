package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.Constants.ClimberConstants;

public class CmdClimbTraversalGyro extends SequentialCommandGroup{

    public CmdClimbTraversalGyro(Climber m_climber, NAR_Drivetrain m_gyro) {
        addCommands(
            

            //Climber is manually fully extended on Mid Bar
            //elev extend a wee bit
            new CmdClimbEncoder(m_climber, 0),

            new WaitCommand(.25),

            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),

            new WaitCommand(0.5),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
            
            new WaitCommand(0.25),

            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            //elev retract
            new CmdClimbEncoder(m_climber, -350),

            //Climber is manually fully retracted on High Bar
            
            new WaitCommand(0.25),
            
            new CmdClimbEncoder(m_climber, m_climber.getDesiredTicks(ClimberConstants.SMALL_VERTICAL_DISTANCE)),
            
            new InstantCommand(() -> m_climber.extendPiston()),
            
            new WaitCommand(0.5),
            
            new CmdIsTraversalAngle(m_gyro),
            
            //elev extend
            new CmdClimbEncoder(m_climber, ClimberConstants.CLIMB_ENC_DIAG_EXTENSION),
            
            //piston extend
            new InstantCommand(() -> m_climber.retractPiston()),

            new WaitCommand(0.5),

            //elev retract
            new CmdClimbEncoder(m_climber, 300000)
        );
    }

}