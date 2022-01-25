package frc.team3128.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants;

public class ClimbSensor extends SequentialCommandGroup{

    public ClimbSensor(Climber m_climber){
        addCommands(
            //Climber is manually fully retracted on Mid Bar
            
            //elev extend a wee bit
            new InstantCommand(() -> m_climber.setDistance(Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE)),
           
            //piston extend
            new InstantCommand(() -> m_climber.extendArm()),
            
            //elev extend
            new ClimbExtend(m_climber),
            
            //piston retract
            new InstantCommand(() -> m_climber.retractArm()),
            
            //elev retract
            new ClimbRetract(m_climber),

            //repeat above commands:
            
            //elev extend a wee bit
            new InstantCommand(() -> m_climber.setDistance(Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE)),
           
            //piston extend
            new InstantCommand(() -> m_climber.extendArm()),
            
            //elev extend
            new ClimbExtend(m_climber),
            
            //piston retract
            new InstantCommand(() -> m_climber.retractArm()),
            
            //elev retract
            new ClimbRetract(m_climber)

        );
    }

}