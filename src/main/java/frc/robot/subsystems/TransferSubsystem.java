/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class TransferSubsystem extends SubsystemBase {
  
  private final VictorSP centerMotor = new VictorSP(HopperConstants.centerMotorPin);
  private final VictorSP topMotor = new VictorSP(HopperConstants.topMotorPin);
  private final VictorSP bottomMotor = new VictorSP(HopperConstants.bottomMotorPin);

  public TransferSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runTransfer(double speed){
    centerMotor.set(speed);
    topMotor.set(speed);
    bottomMotor.set(-speed);
  }

  public void stopTransfer(){
    centerMotor.set(0);
    topMotor.set(0);
    bottomMotor.set(0);
  }
}
