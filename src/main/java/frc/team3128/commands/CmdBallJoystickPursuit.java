package frc.team3128.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants;
import frc.team3128.hardware.input.NAR_Joystick;
import frc.team3128.hardware.limelight.LEDMode;
import frc.team3128.hardware.limelight.Limelight;
import frc.team3128.hardware.limelight.LimelightKey;
import frc.team3128.utility.Log;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdBallJoystickPursuit extends CommandBase {
    private final NAR_Drivetrain m_drivetrain;

    private final Limelight ballLimelight;

    private double multiplier;

    private double currentHorizontalOffset;
    private double previousVerticalAngle;
    private double approxDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;
    
    private double feedbackPower;
    private double leftPower, rightPower;
    private NAR_Joystick m_joystick;

    private double throttle, x, y;
    int targetCount, plateauCount;

    private enum BallPursuitState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private BallPursuitState aimState = BallPursuitState.SEARCHING;
    
    public CmdBallJoystickPursuit(NAR_Drivetrain drive, Limelight ballLimelight, NAR_Joystick joystick) {
        this.ballLimelight = ballLimelight;
        m_drivetrain = drive;
        m_joystick = joystick;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        ballLimelight.setLEDMode(LEDMode.ON);
    }

    @Override
    public void execute() {
        Log.info("State", aimState.toString());
        switch (aimState) {
            case SEARCHING:
                if (ballLimelight.hasValidTarget()) {
                    targetCount ++;
                } else {
                    targetCount = 0;
                    // in an ideal world this would happen after a set time but that is for after testing
                    Log.info("CmdBallPursuit", "No targets ... switching to BLIND...");
                    aimState = BallPursuitState.BLIND;
                }

                if (targetCount > Constants.VisionContants.BALL_THRESHOLD) {
                    Log.info("CmdBallPursuit", "Target found.");
                    Log.info("CmdBallPursuit", "Switching to FEEDBACK...");
                    
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime() / 1e6; 
                    previousError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    aimState = BallPursuitState.FEEDBACK;
                }

                break;
            
            case FEEDBACK:
                if (!ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "No valid target anymore.");
                    aimState = BallPursuitState.SEARCHING;
                } else {
                    currentHorizontalOffset = ballLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime() / 1e6; 
                    currentError = Constants.VisionContants.GOAL_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    // PID feedback loop for left+right powers based on horizontal offset errors
                    feedbackPower = 0;
                    
                    feedbackPower += Constants.VisionContants.BALL_VISION_kP * currentError;
                    feedbackPower += Constants.VisionContants.BALL_VISION_kD * (currentError - previousError) / (currentTime - previousTime);
                    
                    // joystick adding power back/forth
                    throttle = (-m_joystick.getThrottle() + 1) / 2;
                    if (throttle < 0.3) throttle = 0.3;
                    if (throttle > 0.6) throttle = 0.6;
                    x = -m_joystick.getY();

                    leftPower = Math.min(Math.max(0.3 + x*throttle - feedbackPower, -1), 1);
                    rightPower = Math.min(Math.max(0.3 + x*throttle + feedbackPower, -1), 1);
                    
                    // calculations to decelerate as the robot nears the target
                    previousVerticalAngle = ballLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2) * Math.PI / 180;
                    approxDistance = ballLimelight.calculateDistToGroundTarget(previousVerticalAngle, Constants.VisionContants.BALL_TARGET_HEIGHT / 2);
                    SmartDashboard.putNumber("Distance", approxDistance);
                    SmartDashboard.putNumber("Vertical Angle", previousVerticalAngle);

                    // multiplier = 1.0 - Math.min(Math.max((Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - approxDistance)
                    //         / (Constants.VisionContants.BALL_DECELERATE_START_DISTANCE - 
                    //             Constants.VisionContants.BALL_DECELERATE_END_DISTANCE), 0.0), 1.0);
                    multiplier = 1.0;

                    m_drivetrain.tankDrive(0.7*multiplier*leftPower, 0.7*multiplier*rightPower); // bad code - switch to arcadedrive
                    // TODO: switch to arcade drive
                    previousTime = currentTime;
                    previousError = currentError;
                }
                break;
            
            case BLIND:
                throttle = (-m_joystick.getThrottle() + 1) / 2;
                if (throttle < 0.3) throttle = 0.3;
                if (throttle > 0.8) throttle = 1;
                x = -m_joystick.getY()*throttle;
                y = Constants.DriveConstants.ARCADE_DRIVE_TURN_MULT * m_joystick.getTwist()*throttle;
                if (Math.abs(y) < Constants.DriveConstants.ARCADE_DRIVE_TURN_DEADBAND) y = 0;

                m_drivetrain.arcadeDrive(x, y);

                // in an ideal world this would have to find more than one 
                // or maybe that doesn't matter much because it will just go back to blind
                // but maybe two? need testing of initial first
                if (ballLimelight.hasValidTarget()) {
                    Log.info("CmdBallPursuit", "Target found - Switching to SEARCHING");
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
        ballLimelight.setLEDMode(LEDMode.OFF);

        Log.info("CmdBallPursuit", "Command Ended.");
    }

}