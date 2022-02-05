package frc.team3128.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.subsystems.NAR_Drivetrain;


public class CmdAlign extends CommandBase {

    private enum HorizontalOffsetFeedBackDriveState {
        SEARCHING, FEEDBACK;
    }

    private NAR_Drivetrain m_drive;
    private Limelight m_limelight;
    private Set<Subsystem> requirements;

    private double txThreshold = VisionConstants.TX_THRESHOLD;
    private double goalHorizontalOffset, currHorizontalOffset;
    private double prevError, currError;
    
    private double prevTime, currTime; // seconds
    private int plateauCount, targetFoundCount;
    private boolean isAligned;

    private HorizontalOffsetFeedBackDriveState aimState = HorizontalOffsetFeedBackDriveState.SEARCHING;


    public CmdAlign(NAR_Drivetrain drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;
        goalHorizontalOffset = VisionConstants.TX_OFFSET;
        isAligned = false;
        requirements = new HashSet<Subsystem>();

        requirements.add(m_drive);
    }

    @Override
    public void initialize() {
        m_limelight.setLEDMode(LEDMode.ON);
        prevTime = RobotController.getFPGATime() / 1e6;
        plateauCount = 0;
    }

    @Override
    public void execute() {
        currTime = RobotController.getFPGATime() / 1e6;
        switch(aimState) {
            case SEARCHING:
                if(m_limelight.hasValidTarget())
                    targetFoundCount++;
                else
                    targetFoundCount = 0;
                if(targetFoundCount > 5) {
                    currHorizontalOffset = m_limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, VisionConstants.SAMPLE_RATE);
                    prevError = goalHorizontalOffset - currHorizontalOffset;
                    aimState = HorizontalOffsetFeedBackDriveState.FEEDBACK;
                }
                break;
            
            case FEEDBACK:
                if(!m_limelight.hasValidTarget()) {
                    aimState = HorizontalOffsetFeedBackDriveState.SEARCHING;
                    isAligned = false;
                    plateauCount = 0;
                    break;
                }

                currHorizontalOffset = m_limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, VisionConstants.SAMPLE_RATE);
                currError = goalHorizontalOffset - currHorizontalOffset; // currError is positive if we are too far left
                if (txThreshold < VisionConstants.TX_THRESHOLD_MAX) {
                    txThreshold += (currTime - prevTime) * (VisionConstants.TX_THRESHOLD_INCREMENT);
                }

                double ff = Math.signum(currError) * VisionConstants.VISION_PID_kF;
                double feedbackPower = VisionConstants.VISION_PID_kP * currError + VisionConstants.VISION_PID_kD * (currError - prevError) / (currTime - prevTime) + ff;
                
                feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);

                m_drive.tankDrive(-feedbackPower, feedbackPower);
                
                if (Math.abs(currError) < txThreshold) {
                    plateauCount++;
                    if (plateauCount > VisionConstants.ALIGN_PLATEAU_COUNT) {
                        isAligned = true;
                        m_limelight.setLEDMode(LEDMode.OFF);
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
        m_drive.stop();
    }
    
    @Override
    public boolean isFinished() {
        return isAligned;
    }
}