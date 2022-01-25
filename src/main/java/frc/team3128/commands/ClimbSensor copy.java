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
            new RunCommand(
            new RunCommand(m_climber::extendArm, m_climber).withInterrupt(m_climber::getTop),

        );
    }

}