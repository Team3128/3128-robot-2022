package frc.team3128.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallJoystickPursuit extends CommandBase {

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private NAR_Drivetrain m_drivetrain;
    private Limelight ballLimelight;

    private double powerMult = 0.7;

    private double previousVerticalAngle;
    private double approxDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private DoubleSupplier xSupplier, ySupplier, throttleSupplier;

    private int targetCount, plateauCount;

    private BallPursuitState aimState = BallPursuitState.SEARCHING;

    
    public CmdBallJoystickPursuit(NAR_Drivetrain drive, Limelight ballLimelight, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier throttleSupplier) {
        
        m_drivetrain = drive;
        this.ballLimelight = ballLimelight;
        
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.throttleSupplier = throttleSupplier;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Log.info("CmdBallJoystickPursuit State", aimState.toString());
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount++;
                } else {
                    targetCount = 0;
                    Log.info("CmdBallJoystickPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > VisionConstants.BALL_THRESHOLD) {
                    Log.info("CmdBallJoystickPursuit", "Target found.");
                    Log.info("CmdBallJoystickPursuit", "Switching to FEEDBACK...");
                    
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = VisionConstants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallJoystickPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } 
                else {
                    double currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = VisionConstants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    double feedbackPower = 0;
                    
                    feedbackPower += VisionConstants.BALL_VISION_kP * currentError;
                    feedbackPower += VisionConstants.BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);

                    feedbackPower = MathUtil.clamp(feedbackPower, -1, 1);
                    
                    // joystick adding power back/forth
                    double throttle = throttleSupplier.getAsDouble();
                    double x = xSupplier.getAsDouble();

                    double forwardPower = MathUtil.clamp(VisionConstants.BALL_AUTO_PURSUIT_kF + x * throttle, -1, 1);
                    
                    // calculations to decelerate as the robot nears the target
                    approxDistance = ballLimelight.calculateDistToGroundTarget(VisionConstants.BALL_TARGET_HEIGHT / 2);

                    // multiplier = 1.0 - Math.min(Math.max((Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - approxDistance)
                    //         / (Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - 
                    //             Constants.VisionContants.BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);
                    double multiplier = powerMult * 1.0;

                    m_drivetrain.arcadeDrive(forwardPower * multiplier, powerMult * feedbackPower);

                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:
                double throttle = throttleSupplier.getAsDouble();
                double x = xSupplier.getAsDouble();
                double y = DriveConstants.ARCADE_DRIVE_TURN_MULT * ySupplier.getAsDouble();

                m_drivetrain.arcadeDrive(x * throttle, y * throttle);


                if (ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallJoystickPursuit", "Target found - Switching to SEARCHING");
                    aimState = BallPursuitState.SEARCHING;
                }
                    
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();

        Log.info("CmdBallJoystickPursuit", "Command Ended.");
    }

}