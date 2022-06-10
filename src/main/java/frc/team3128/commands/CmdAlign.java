package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdAlign extends CommandBase {

    private enum VisionState {
        SEARCHING, FEEDBACK;
    }

    private NAR_Drivetrain drive;
    private LimelightSubsystem limelights;

    private double goalHorizontalOffset, currHorizontalOffset;
    private double prevError, currError;
    
    private double prevTime, currTime; // seconds
    private int plateauCount, targetFoundCount;

    private VisionState aimState = VisionState.SEARCHING;

    private boolean isAligned;

    /**
     * Aligns the robot to the hub using the limelight
     * @Requirements Drivetrain  
     */
    public CmdAlign() {
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
                if (limelights.getShooterHasValidTarget())
                    targetFoundCount++;
                else
                    targetFoundCount = 0;
                
                // if target found for enough iterations, switch to FEEDBACK
                if(targetFoundCount > 5) {
                    currHorizontalOffset = limelights.getShooterTX();
                    prevError = goalHorizontalOffset - currHorizontalOffset;
                    aimState = VisionState.FEEDBACK;
                }
                break;
            
            case FEEDBACK:
                // if no more valid target, switch to SEARCHING
                if(!limelights.getShooterHasValidTarget()) {
                    aimState = VisionState.SEARCHING;
                    plateauCount = 0;
                    break;
                }

                // turn with PID loop using input as horizontal tx error to target
                currHorizontalOffset = limelights.getShooterTX();
                currError = goalHorizontalOffset - currHorizontalOffset; // currError is positive if we are too far left

                double ff = Math.signum(currError) * VISION_PID_kF;
                double feedbackPower = VISION_PID_kP * currError + VISION_PID_kD * (currError - prevError) / (currTime - prevTime) + ff;
                
                feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);

                drive.tankDrive(-feedbackPower, feedbackPower);
                prevTime = currTime;

                if (Math.abs(currError) < TX_THRESHOLD) {
                    plateauCount++;
                    if (plateauCount > ALIGN_PLATEAU_COUNT) {
                        SmartDashboard.putBoolean("Shooter isAligned", true);
                        isAligned = true;
                    }
                }
                else {
                    plateauCount = 0;
                    isAligned = false;
                }
                SmartDashboard.putBoolean("Shooter isAligned", false);
                prevError = currError;

                break;
                
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        // limelights.turnShooterLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        // if degrees of horizontal tx error below threshold (aligned enough)
        return isAligned;
    }
}