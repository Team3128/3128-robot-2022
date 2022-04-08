/**
 * I'm moving our old autos that we're very likely not using ever here. 
 * However, I am lazy and didn't want to do it properly, so this entire 
 * file is commented out. We will decide whether to redo this or simply 
 * get rid of them when we clean up our codebase.
 */


// package frc.team3128.autonomous;

// public class OldAutos {

    /**
     * The initial positions will be a pain to get back if we so choose to put these back.
     */
    // initialPoses.put(auto_2BallBot, trajectory[0].getInitialPose());
    // initialPoses.put(auto_2BallMid, trajectory[1].getInitialPose());
    // initialPoses.put(auto_2BallTop, trajectory[2].getInitialPose());
    // initialPoses.put(auto_3BallHook, trajectory[3].getInitialPose());
    // initialPoses.put(auto_3BallHersheyKiss, trajectory[5].getInitialPose());
    // initialPoses.put(auto_3BallTerminal, trajectory[7].getInitialPose());
    // initialPoses.put(auto_4BallE, trajectory[11].getInitialPose());
    // initialPoses.put(auto_4BallTerm, trajectory[13].getInitialPose());
    // initialPoses.put(auto_5Ball, trajectory[15].getInitialPose());
    // initialPoses.put(auto_3BallBack, trajectory[23].getInitialPose());


//     //no use
//     auto_2BallBot = new SequentialCommandGroup(

//         //pick up 1 ball
//         new ParallelDeadlineGroup(
//             trajectoryCmd(0).andThen(m_drive::stop, m_drive),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot preloaded + first
//         shootCmd(3250)

// );

// //no use
// auto_2BallMid = new SequentialCommandGroup(

//          //pick up 1 ball
//         new ParallelDeadlineGroup(
//             trajectoryCmd(1).andThen(m_drive::stop, m_drive),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot first + preloaded
//         shootCmd(3250)

// );

// auto_2BallTop = new SequentialCommandGroup(
        
//         new WaitCommand(2),

//         //pick up 1 ball

//         new CmdExtendIntake(m_intake),

//         new ParallelDeadlineGroup(
//             trajectoryCmd(2).andThen(m_drive::stop, m_drive),
//             new CmdIntakeCargo(m_intake, m_hopper)
//         ),

//         new InstantCommand(() -> m_intake.retractIntake()),

//         new CmdInPlaceTurn(m_drive, 85),

//         //shoot first + preloaded
//         new InstantCommand(() -> m_shooterLimelight.turnLEDOn()),
//         alignShootCmd() // 3750

// );

// //no use
// auto_3BallHook = new SequentialCommandGroup(

//         //shoot preloaded ball
//         shootCmd(3350),

//         //pick up two balls
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(3),
//                 trajectoryCmd(4),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot two balls
//         shootCmd(3250)

// );

// //no use
// auto_3BallHersheyKiss = new SequentialCommandGroup(

//         //shoot preload
//         shootCmd(3500),
        
//         //pick up two balls
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(5),
//                 trajectoryCmd(6),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot two balls
//         alignShootCmd()
// );

// //no use
// auto_3BallTerminal = new SequentialCommandGroup(

//         //shoot preloaded ball
//         shootCmd(3000),

//         trajectoryCmd(7),
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(8),
//                 new InstantCommand(m_drive::stop, m_drive),
//                 new WaitCommand(1)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),
        
//         trajectoryCmd(19),
//         trajectoryCmd(20),
//         new InstantCommand(m_drive::stop, m_drive),

//         //shoot two balls
//         shootCmd(3000)

// );

// //no use
// auto_3BallBack = new SequentialCommandGroup(
//     shootCmd(3500),

//     new CmdExtendIntake(m_intake),
//     new WaitCommand(0.1), // slow but it might work
//     new ParallelDeadlineGroup(
//         new SequentialCommandGroup(
//             trajectoryCmd(23),
//             trajectoryCmd(24)
//         ),
//         new CmdIntakeCargo(m_intake,m_hopper)
//     ),
//     new InstantCommand(() -> m_intake.retractIntake()),

//     alignShootCmd()
// );

// //no use
// auto_4BallE = new SequentialCommandGroup(

//         //pick up first ball
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(

//                 trajectoryCmd(11),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot first + preloaded
//         shootCmd(3000),

//         //pick up two more balls
//         new CmdExtendIntake(m_intake).withTimeout(0.1),
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(12),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         //shoot two more balls
//         shootCmd(3250)

// );

// //no use
// auto_4BallTerm = new SequentialCommandGroup(
//         //pick up first ball

//         new CmdExtendIntake(m_intake),

//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(13),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdIntakeCargo(m_intake, m_hopper)
//         ),

//         new CmdRetractHopper(m_hopper),

//         shootCmd(3750),
//         new CmdExtendIntake(m_intake),

//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(14),
//                 new InstantCommand(m_drive::stop, m_drive),
//                 new WaitCommand(0.5)
//             ),
//             new CmdIntakeCargo(m_intake, m_hopper)
//         ),

//         new CmdRetractHopper(m_hopper),

//         trajectoryCmd(15),
//         trajectoryCmd(16),
//         new InstantCommand(m_drive::stop, m_drive),

//         //shoot two balls
//         shootCmd(3750)
// );

// //no use
// auto_5Ball = new SequentialCommandGroup(

//         new SequentialCommandGroup(
//             new CmdRetractHopper(m_hopper).withTimeout(0.5),
//             new ParallelCommandGroup(
//                 new CmdHopperShooting(m_hopper, m_shooter::isReady),
//                 new CmdShootRPM(m_shooter, 3250)
//             ).withTimeout(1)
//         ),

//         //pick up first ball
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(15),
//                 trajectoryCmd(16),
//                 new InstantCommand(m_drive::stop, m_drive)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         shootCmd(3750),
        
//         trajectoryCmd(17),

//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(
//                 trajectoryCmd(18),
//                 new InstantCommand(m_drive::stop, m_drive),
//                 new WaitCommand(0.5)
//             ),
//             new CmdExtendIntakeAndRun(m_intake, m_hopper)
//         ),

//         trajectoryCmd(19),
//         trajectoryCmd(20),
//         new InstantCommand(m_drive::stop, m_drive),

//         //shoot two balls
//         shootCmd(3750)
// );
// }

