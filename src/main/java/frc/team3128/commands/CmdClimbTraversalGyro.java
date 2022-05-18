package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.NAR_Drivetrain;
import static frc.team3128.Constants.ClimberConstants.*;
import frc.team3128.Constants.ClimberConstants;

public class CmdClimbTraversalGyro extends SequentialCommandGroup{
    private Climber m_climber;

    public CmdClimbTraversalGyro() {
        m_climber = Climber.getInstance();

        addCommands(

            //Climber is manually fully extended on Mid Bar
            //elev extend a wee bit
            new CmdClimbEncoder(0),

            new WaitCommand(.25),

            new CmdClimbEncoder(m_climber.getDesiredTicks(SMALL_VERTICAL_DISTANCE)),

            //piston extend
            new InstantCommand(() -> m_climber.extendPiston()),
            
            //elev extend
            new CmdClimbEncoder(CLIMB_ENC_DIAG_EXTENSION),
            
            new WaitCommand(0.25),

            //piston retract
            new InstantCommand(() -> m_climber.retractPiston()),
            
            //elev retract
            new CmdClimbEncoder(-350),

            //Climber is manually fully retracted on High Bar
            
            new WaitCommand(0.25),
            
            new CmdClimbEncoder(m_climber.getDesiredTicks(SMALL_VERTICAL_DISTANCE)),
            
            new InstantCommand(() -> m_climber.extendPiston()),
            
            new CmdIsTraversalAngle(),
            
            //elev extend
            new CmdClimbEncoder(CLIMB_ENC_DIAG_EXTENSION),
            
            //piston extend
            new InstantCommand(() -> m_climber.retractPiston()),

            //elev retract
            new CmdClimbEncoder(300000)
        );
    }

}