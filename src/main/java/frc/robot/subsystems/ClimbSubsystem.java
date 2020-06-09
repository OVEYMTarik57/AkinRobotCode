/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX climbMotor = new WPI_VictorSPX(ClimbConstants.climbMotorPin);
  private final Servo servoMotor = new Servo(ClimbConstants.servoMotorPin);

  public ClimbSubsystem() {

  }

  @Override
  public void periodic() {
    
    }

    public void runClimb(double speed){
      climbMotor.set(speed);
    }

    public void runServo(double speed){ //servo motoru çalıştırıyoruz.
      servoMotor.set(speed);
    }

    public void stopServo(){ 
      servoMotor.set(0);
    } 

    public void stopClimb(){
      climbMotor.set(0);
    }

  }
