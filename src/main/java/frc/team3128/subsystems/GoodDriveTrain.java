package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.hardware.*;
import frc.team3128.Robot;

public class GoodDriveTrain extends SubsystemBase {
    
    private static BaseTalon m_leftMotor;
    private static BaseTalon m_rightMotor;
    private static TalonSRXSimCollection m_leftMotorSim;
    private static TalonSRXSimCollection m_rightMotorSim;
    private static DifferentialDrive m_robotDrive;
    private static DifferentialDrivetrainSim m_robotDriveSim;
    private static DifferentialDriveOdometry m_odometry;

    private static AHRS m_gyro;

    private static Field2d m_field;

    public GoodDriveTrain(){

        m_robotDrive = new DifferentialDrive((GoodTalonFX)m_leftMotor, (GoodTalonFX)m_rightMotor);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        m_gyro = new AHRS();

        if(Robot.isReal()){
            m_leftMotor = new GoodTalonFX(0);
            m_rightMotor = new GoodTalonFX(1);
            m_robotDrive = new DifferentialDrive((GoodTalonFX)m_leftMotor, (GoodTalonFX)m_rightMotor);
          }else{
            m_leftMotor = new WPI_TalonSRX(0);
            m_rightMotor = new WPI_TalonSRX(1);
            m_leftMotorSim = new TalonSRXSimCollection(m_leftMotor);
            m_rightMotorSim = new TalonSRXSimCollection(m_rightMotor);

            m_field = new Field2d();

            // This class simulates our drivetrain's motion around the field.
            m_robotDriveSim =
            new DifferentialDrivetrainSim(
                LinearSystemId.identifyDrivetrainSystem(
                   0.5, // kvVoltSecondsPerMeter
                   0.05, // kaVoltSecondsSquaredPerMeter
                   1.5, // kvVoltSecondsPerRadian
                   0.3 // kaVoltSecondsSquaredPerRadian
                ),
                DCMotor.getCIM(2), // gearbox
                8, // driveGearing
                0.66, // trackWidthMeters
                2, // wheelRadius
                null/*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);
          }
    }
    
    public double getHeading() {
      return 0;
    }

    public void arcadeDrive(double x, double y){
      m_robotDrive.arcadeDrive(x,y);
    }
}
