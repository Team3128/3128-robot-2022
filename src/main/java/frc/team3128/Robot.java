// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.hardware.*;
import frc.team3128.subsystems.*;
import frc.team3128.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
 
  private Mover m_move; // test subsystem
  private GoodDriveTrain m_drive;
  private Joystick m_stick;

  @Override
  public void robotInit(){
    m_stick = new Joystick(0);
    m_move = new Mover();
    m_drive = new GoodDriveTrain();
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
