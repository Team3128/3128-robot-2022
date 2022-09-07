package frc.team3128.commands;

import frc.team3128.commands.CmdAlign_Feedback;
import frc.team3128.commands.CmdAlign_Search;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.team3128.Constants.VisionConstants.*;


import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdAlign extends PIDCommand {

    private enum VisionState {
        SEARCHING, FEEDBACK;
    }

    protected NAR_Drivetrain drive;
    protected LimelightSubsystem limelights;

    protected double goalHorizontalOffset, currHorizontalOffset;
    protected double prevError, currError;
    
    protected double prevTime, currTime; // seconds
    protected int plateauCount, targetFoundCount;
    protected boolean isAligned;

    protected VisionState aimState = VisionState.SEARCHING;

    private CmdAlign_Feedback cmdFeedback;
    private CmdAlign_Search cmdSearch;

    /**
     * Aligns the robot to the hub using the limelight
     * @Requirements Drivetrain  
     */
    public CmdAlign() {
        super
        (
            new PIDController(VISION_PID_kP, VISION_PID_kI, VISION_PID_kD),
            NAR_Drivetrain.getInstance()::getHeading,
            0, // setpoint initialized in initalize
            output -> NAR_Drivetrain.getInstance().tankDrive(output + Math.copySign(VISION_PID_kF, output),
            -output - Math.copySign(VISION_PID_kF, output))
        );
        this.drive = NAR_Drivetrain.getInstance();
        this.limelights = LimelightSubsystem.getInstance();

        goalHorizontalOffset = TX_OFFSET;
        isAligned = false;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // limelights.turnShooterLEDOn();
        prevTime = RobotController.getFPGATime() / 1e6;
        plateauCount = 0;
    }

    @Override
    public void execute() {
        currTime = RobotController.getFPGATime() / 1e6;
        switch (aimState) {
            case SEARCHING:
                cmdSearch = new CmdAlign_Search();
                if (cmdSearch != null) {
                    cmdSearch.schedule();
                }
                break;
            
            case FEEDBACK:
                cmdFeedback = new CmdAlign_Feedback();
                if (cmdFeedback != null) {
                    cmdFeedback.schedule();
                }
                break;
                
        }
        prevTime = currTime;
        SmartDashboard.putBoolean("Shooter isAligned", isAligned);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        // limelights.turnShooterLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return isAligned;
    }
}