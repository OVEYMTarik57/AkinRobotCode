/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  public boolean isAtSetpoint = false;
  
  private final VictorSP shooterMotor1 = new VictorSP(ShooterConstants.shooterMotor1Pin);
  private final VictorSP shooterMotor2 = new VictorSP(ShooterConstants.shooterMotor2Pin);
  public final Encoder shooterEncoder = new Encoder(ShooterConstants.kShooterEncoderA,
      ShooterConstants.kShooterEncoderB, ShooterConstants.kShooterEncoderIsReversed);



  public ShooterSubsystem() {
    shooterEncoder.setDistancePerPulse(1.0 / (ShooterConstants.kShooterEncoderPPR));
    
    //shooterMotor2.follow(shooterMotor1); //sadece  motor 1 i kullansaydım takip ettirmem gerekirdi...

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter/atSetpoint", isAtSetpoint);
    SmartDashboard.putNumber("shooter/RPM", getRPM()); // Smart Dashboard veri aktarımı...
   
  }

  public void runShooter(double speed){

    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }
  public void stopShooter(){

    shooterMotor1.set(0);
    shooterMotor2.set(0);
  }

  public double getRPM() {
    return shooterEncoder.getRate() * 60;
  }

  public void runShooterVoltage(double voltage) {
    shooterMotor1.setVoltage(voltage);
  }

}
