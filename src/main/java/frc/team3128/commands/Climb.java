package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Climber;
import frc.team3128.Constants;

public class Climb extends SequentialCommandGroup{

    public Climb(Climber m_climber){
        addCommands(
            new ClimbVertical(m_climber, Constants.ClimberConstants.VERTICAL_DISTANCE)

        );
    }

}