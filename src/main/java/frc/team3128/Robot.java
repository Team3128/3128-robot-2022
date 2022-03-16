// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

    public static RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    // private Thread dashboardUpdateThread;

    // private ArrayList<Double> battVoltages = new ArrayList<Double>();
    // public static double voltageRollingAvg = 0;

    // AddressableLED m_led;
    // AddressableLEDBuffer m_ledBuffer;
    // int m_rainbowFirstPixelHue = 0;

    @Override
    public void robotInit(){
        LiveWindow.disableAllTelemetry();
        //CameraServer.startAutomaticCapture();

        // m_led = new AddressableLED(9);

        // m_ledBuffer = new AddressableLEDBuffer(60);
        // m_led.setLength(m_ledBuffer.getLength());

        // // Set the data
        // m_led.setData(m_ledBuffer);
        // m_led.start();
    }

    @Override
    public void robotPeriodic(){
        m_robotContainer.updateDashboard();

        // if(battVoltages.size() == 100) {
        //     battVoltages.remove(0);
        // }
        // battVoltages.add(RobotController.getBatteryVoltage());

        // for (double d : battVoltages) {
        //     voltageRollingAvg += d;
        // }
        // voltageRollingAvg /= battVoltages.size();
        // rainbow();
        
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.init();
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
        m_robotContainer.init();
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

    // private void rainbow() {
    //     // For every pixel
    //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //       // Calculate the hue - hue is easier for rainbows because the color
    //       // shape is a circle so only one value needs to precess
    //       final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
    //       // Set the value
    //       m_ledBuffer.setHSV(i, hue, 255, 128);
    //     }
    //     // Increase by to make the rainbow "move"
    //     m_rainbowFirstPixelHue += 3;
    //     // Check bounds
    //     m_rainbowFirstPixelHue %= 180;
    //   }
}
