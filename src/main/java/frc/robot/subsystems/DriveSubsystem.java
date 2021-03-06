/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */
    private final WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(Constants.DriveConstants.frontLeftMotorPin); 
    private final WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(Constants.DriveConstants.rearLeftMotorPin);
    private final WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(Constants.DriveConstants.frontRightMotorPin);
    private final WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(Constants.DriveConstants.rearRightMotorPin);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
    private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);


    private final Encoder rightWheelEncoder = new Encoder(Constants.DriveConstants.rightWheelEncoder_A,DriveConstants.rightWheelEncoder_B,false,EncodingType.k4X);
    private final Encoder leftWheelEncoder = new Encoder(Constants.DriveConstants.leftWheelEncoder_A,DriveConstants.leftWheelEncoder_B,false,EncodingType.k4X);

    private final DifferentialDriveOdometry m_odometry;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();


  public DriveSubsystem() {
    rightWheelEncoder.setDistancePerPulse((7.62*2*Math.PI)/2048.0);
    leftWheelEncoder.setDistancePerPulse((7.62*2*Math.PI)/2048.0);
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    gyro.calibrate();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftWheelEncoder.getDistance(),
                      rightWheelEncoder.getDistance());
    
  }

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot, true);
  } 
  
  
  public double getRightWheelCm(){
    return rightWheelEncoder.getDistance();
  }
  
  
  public double getLeftWheelCm(){
    return leftWheelEncoder.getDistance();
  }
  
  
  public double  readYawAngle()
  {
    return gyro.getAngle();
  }
  
  
    public double getHeading(){
    return Math.IEEEremainder(-1*gyro.getAngle(), 360);
  }

  
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftWheelEncoder.getRate(), rightWheelEncoder.getRate());
  }
  
  
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(-rightVolts);
    m_drive.feed();
  }


}
