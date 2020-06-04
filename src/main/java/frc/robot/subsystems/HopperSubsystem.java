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


public class HopperSubsystem extends SubsystemBase {
  
  private final VictorSP hopperBagMotor = new VictorSP(HopperConstants.hopperBagMotorPin);

  public HopperSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runHopper(double speed){
    hopperBagMotor.set(speed);
  }

  public void stopHopper(){
    hopperBagMotor.set(0);
  }

}
