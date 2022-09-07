package frc.team3128.commands;

import static frc.team3128.Constants.VisionConstants.*;

import edu.wpi.first.math.MathUtil;


public class CmdAlign_Feedback extends CmdAlign{
    private boolean searching;
    private CmdAlign_Search searchCommand;

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        // if no more valid target, switch to SEARCHING
        if(!limelights.getShooterHasValidTarget()) {
            plateauCount = 0;
            searching = true;
        }

        // turn with PID loop using input as horizontal tx error to target
        currHorizontalOffset = limelights.getShooterTX();
        currError = goalHorizontalOffset - currHorizontalOffset; // currError is positive if we are too far left

        double ff = Math.signum(currError) * VISION_PID_kF;
        double feedbackPower = VISION_PID_kP * currError + VISION_PID_kD * (currError - prevError) / (currTime - prevTime) + ff;
        
        feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);

        drive.tankDrive(-feedbackPower, feedbackPower);

        // if degrees of horizontal tx error below threshold (aligned enough)
        if (Math.abs(currError) < TX_THRESHOLD) {
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

    }

    @Override
    public void end(boolean interrupted)
    {
        if (searching)
        {
            searchCommand = new CmdAlign_Search();
            if (searchCommand != null) {
                searchCommand.schedule();
            }
        }
        if (isAligned)
        {
            super.schedule();
        }
    }

    @Override
    public boolean isFinished()
    {
        if(searching) {
            return true;
        }
        return isAligned;
    }
}
