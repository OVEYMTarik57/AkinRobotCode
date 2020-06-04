/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private final VictorSP intakeMotor = new VictorSP (IntakeConstants.intakeMotorPin); 
  private final VictorSP intakeOpen1Motor = new VictorSP(IntakeConstants.intakeOpen1MotorPin); 
  private final VictorSP intakeOpen2Motor = new VictorSP(IntakeConstants.intakeOpen2MotorPin); 
  private final DigitalInput intakeHallEffect = new DigitalInput(IntakeConstants.intakeHallEffect);
  
  

  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed){
    intakeMotor.set(speed);
  } 
  
  public void stopIntake(){
    intakeMotor.set(0);
 
  }

  public void intakeOpen(double speed){
    intakeOpen1Motor.set(speed);
    intakeOpen2Motor.set(-speed);

  }

  public void intakeClose(double speed){
    intakeOpen1Motor.set(-speed);
    intakeOpen2Motor.set(speed); 
  }

  public boolean getHallEffect(){
    return intakeHallEffect.get();
  }
  

}

