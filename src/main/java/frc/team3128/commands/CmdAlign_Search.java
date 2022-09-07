package frc.team3128.commands;

public class CmdAlign_Search extends CmdAlign{
    private boolean found;

    @Override
    public void initialize()
    {
        found = false;
    }

    @Override
    public void execute()
    {
        if (limelights.getShooterHasValidTarget())
            targetFoundCount++;
        else
            targetFoundCount = 0;
        
        // if target found for enough iterations, switch to FEEDBACK
        if(targetFoundCount > 5) {
            currHorizontalOffset = limelights.getShooterTX();
            prevError = goalHorizontalOffset - currHorizontalOffset;
            found = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        super.schedule();
    }

    @Override
    public boolean isFinished()
    {
        return found;
    }
}
