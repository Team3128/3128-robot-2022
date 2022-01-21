// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

    public static RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    private Thread dashboardUpdateThread;

    @Override
    public void robotInit(){
        LiveWindow.disableAllTelemetry();
        //Log.info("NarwhalRobot", "Starting Dashboard Update Thread...");
        dashboardUpdateThread = new Thread(this::updateDashboardLoop, "Dashboard Update Thread");
        dashboardUpdateThread.start();
    }

    @Override
    public void robotPeriodic(){
        
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.stopDrivetrain();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledPeriodic() {

    }

    // Narwhal dash stuff (not sure if should be here)

    private void updateDashboard(){
        NarwhalDashboard.put("time", Timer.getFPGATimestamp());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
    }

    private void updateDashboardLoop() {
        //Log.info("NarwhalRobot", "Dashboard Update Thread starting");
        while (true) {
            updateDashboard();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                //Log.info("NarwhalRobot", "Dashboard Update Thread shutting down");
                return;
            }
        }
    }
}
