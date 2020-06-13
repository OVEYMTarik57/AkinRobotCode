/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX (IntakeConstants.intakeMotorPin); 
  private final WPI_VictorSPX intakeOpen1Motor = new WPI_VictorSPX(IntakeConstants.intakeOpen1MotorPin); 
  private final WPI_VictorSPX intakeOpen2Motor = new WPI_VictorSPX(IntakeConstants.intakeOpen2MotorPin); 
  private final DigitalInput intakeTopHallEffect = new DigitalInput(IntakeConstants.intakeTopHallEffect);
  private final DigitalInput intakeBottomHallEffect = new DigitalInput(IntakeConstants.intakeBottomHallEffect);
  
  

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

  public boolean getTopHallEffect(){
    return intakeTopHallEffect.get();
  }

  public boolean getBottomHallEffect(){
    return intakeBottomHallEffect.get();
  }
  

}

