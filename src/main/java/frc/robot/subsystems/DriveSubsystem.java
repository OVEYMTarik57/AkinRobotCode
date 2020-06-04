/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */
    private final VictorSP frontLeftMotor = new VictorSP(Constants.DriveConstants.frontLeftMotorPin); 
    private final VictorSP rearLeftMotor = new VictorSP(Constants.DriveConstants.rearLeftMotorPin);
    private final VictorSP frontRightMotor = new VictorSP(Constants.DriveConstants.frontRightMotorPin);
    private final VictorSP rearRightMotor = new VictorSP(Constants.DriveConstants.rearRightMotorPin);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
    private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ArcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot, true);
  } 
  

}
