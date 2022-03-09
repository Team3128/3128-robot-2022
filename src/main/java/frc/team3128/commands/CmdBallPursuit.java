package frc.team3128.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallPursuit extends CommandBase {

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private NAR_Drivetrain m_drivetrain;
    private Limelight ballLimelight;

    private double powerMult = 0.7;

    private double previousVerticalAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;

    int targetCount, plateauCount;

    private BallPursuitState aimState = BallPursuitState.SEARCHING;

    
    public CmdBallPursuit(NAR_Drivetrain drive, Limelight ballLimelight) {
        this.ballLimelight = ballLimelight;
        m_drivetrain = drive;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Log.info("State", aimState.toString());
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > VisionConstants.BALL_THRESHOLD) {
                    Log.info("CmdBallPursuit", "Target found.");
                    Log.info("CmdBallPursuit", "Switching to FEEDBACK...");
                    
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = VisionConstants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } else {
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = VisionConstants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    double feedbackPower = 0;
                    
                    feedbackPower += VisionConstants.BALL_VISION_kP * currentError;
                    feedbackPower += VisionConstants.BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);
                    
                    feedbackPower = Math.min(Math.max(feedbackPower, -1), 1);
                    
                    // calculations to decelerate as the robot nears the target
                    double approxDistance = ballLimelight.calculateDistToGroundTarget(VisionConstants.BALL_TARGET_HEIGHT / 2);

                    double multiplier = 1.0 - Math.min(Math.max((VisionConstants.BALL_DECELERATE_START_DISTANCE - approxDistance)
                            / (VisionConstants.BALL_DECELERATE_START_DISTANCE - 
                                VisionConstants.BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);
                    

                    m_drivetrain.arcadeDrive(powerMult * VisionConstants.BALL_VISION_kF * multiplier, powerMult * feedbackPower);
                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:

                m_drivetrain.tankDrive(0.35, -0.35); // in-place turn

                if (ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "Target found - Switching to SEARCHING");
                    aimState = BallPursuitState.SEARCHING;
                }
                    
                break;
        }
    }

    @Override
    public boolean isFinished() {
        /*
        When the robot is moving very slowly + blind the robot has probably just intook (?)
        since it decelerated and there is no more target the limelight sees.
        */
        if (aimState == BallPursuitState.BLIND) {
            double leftVel = Math.abs(m_drivetrain.getLeftEncoderSpeed());
            double rightVel = Math.abs(m_drivetrain.getRightEncoderSpeed());

            if (leftVel < VisionConstants.BALL_VEL_THRESHOLD && rightVel < VisionConstants.BALL_VEL_THRESHOLD) {
                plateauCount += 1;
            } else {
                plateauCount = 0;
            }

            if (plateauCount >= VisionConstants.BALL_VEL_PLATEAU_THRESHOLD) {
                return false;
            }
        }

        // if in feedback and only 1 ball length away, stop command & give control back to teleop (but not if ball is bouncing and messing up the math)
        if (aimState == BallPursuitState.FEEDBACK) {
            double approxDistance = ballLimelight.calculateDistToGroundTarget(VisionConstants.BALL_TARGET_HEIGHT / 2);
            if (VisionConstants.BALL_DECELERATE_END_DISTANCE > approxDistance && previousVerticalAngle < 20*Math.PI/180) {
                Log.info("CmdBallPursuit", "decelerated! ending command now");
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();

        Log.info("CmdBallPursuit", "Command Ended.");
    }

}