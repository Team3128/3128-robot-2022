// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.subsystems.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
 
  private Mover m_move; // test subsystem
  private GoodDriveTrain m_drive;
  private Joystick m_stick;
  private Shooter m_shooter;

  @Override
  public void robotInit(){
    m_stick = new Joystick(0);
    m_move = new Mover();
    m_drive = new GoodDriveTrain();
    m_shooter = new Shooter();
  }

  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    m_drive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  @Override
  public void teleopPeriodic() {
    m_drive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  @Override
  public void simulationInit(){
    
  }
}
