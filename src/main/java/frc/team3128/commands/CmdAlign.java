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

    private double txThreshold = TX_THRESHOLD;
    private double goalHorizontalOffset, currHorizontalOffset;
    private double prevError, currError;
    
    private double prevTime, currTime; // seconds
    private int plateauCount, targetFoundCount;
    private boolean isAligned;

    private VisionState aimState = VisionState.SEARCHING;


    public CmdAlign() {
        this.drive = NAR_Drivetrain.getInstance();
        this.limelights = LimelightSubsystem.getInstance();

        goalHorizontalOffset = TX_OFFSET;
        isAligned = false;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        limelights.turnShooterLEDOn();
        prevTime = RobotController.getFPGATime() / 1e6;
        plateauCount = 0;
    }

    @Override
    public void execute() {
        currTime = RobotController.getFPGATime() / 1e6;
        switch (aimState) {
            case SEARCHING:
                if(limelights.getShooterHasValidTarget())
                    targetFoundCount++;
                else
                    targetFoundCount = 0;
                if(targetFoundCount > 5) {
                    currHorizontalOffset = limelights.getShooterTX();
                    prevError = goalHorizontalOffset - currHorizontalOffset;
                    aimState = VisionState.FEEDBACK;
                }
                break;
            
            case FEEDBACK:
                if(!limelights.getShooterHasValidTarget()) {
                    aimState = VisionState.SEARCHING;
                    plateauCount = 0;
                    break;
                }

                currHorizontalOffset = limelights.getShooterTX();
                currError = goalHorizontalOffset - currHorizontalOffset; // currError is positive if we are too far left

                double ff = Math.signum(currError) * VISION_PID_kF;
                double feedbackPower = VISION_PID_kP * currError + VISION_PID_kD * (currError - prevError) / (currTime - prevTime) + ff;
                
                feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);

                drive.tankDrive(-feedbackPower, feedbackPower);
                
                if (Math.abs(currError) < txThreshold) {
                    plateauCount++;
                    if (plateauCount > ALIGN_PLATEAU_COUNT) {
                        isAligned = true;
                    }
                }
                else {
                    isAligned = false;
                    plateauCount = 0;
                }

                prevError = currError;

                break;
                
        }
        prevTime = currTime;
        SmartDashboard.putBoolean("Shooter isAligned", isAligned);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        limelights.turnShooterLEDOff();
    }
    
    @Override
    public boolean isFinished() {
        return isAligned;
    }
}