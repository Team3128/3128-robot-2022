// package frc.team3128.commands;


// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.team3128.subsystems.Climber;
// import frc.team3128.Constants;

// public class ClimbEncoder extends SequentialCommandGroup{

//     public ClimbEncoder(Climber m_climber){
//         addCommands(
//             //Climber is manually fully retracted on Mid Bar
//             new InstantCommand(() -> m_climber.setDistance(Constants.ClimberConstants.SMALL_VERTICAL_DISTANCE)),
//             new InstantCommand(() -> m_climber.extendArm()),
//             new InstantCommand(() -> m_climber.setDistance(Constants.ClimberConstants.ANGLED_DISTANCE)),
//             new InstantCommand(() -> m_climber.retractArm()),
//             new InstantCommand(() -> m_climber.setDistance(-Constants.ClimberConstants.ANGLED_DISTANCE))

//         );
//     }

// }