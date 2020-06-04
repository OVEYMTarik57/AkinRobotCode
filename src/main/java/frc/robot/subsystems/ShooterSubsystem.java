/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  private final VictorSP shooterMotor1 = new VictorSP(ShooterConstants.shooterMotor1Pin);
  private final VictorSP shooterMotor2 = new VictorSP(ShooterConstants.shooterMotor2Pin);
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
   
  }

  public void runShooter(double speed){

    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }
  public void stopShooter(){

    shooterMotor1.set(0);
    shooterMotor2.set(0);
  }
  

}
