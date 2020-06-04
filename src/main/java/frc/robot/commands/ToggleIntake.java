/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends CommandBase {
  private final IntakeSubsystem m_toggleintake;
  private final double m_speed;
  
  public ToggleIntake(IntakeSubsystem toggleintake, double speed) {
    m_toggleintake = toggleintake;
    m_speed = speed;
    addRequirements(m_toggleintake);
    
  }

  
  @Override
  public void initialize() {
  }

 
  @Override
  public void execute() {
    m_toggleintake.intakeOpen(m_speed);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_toggleintake.intakeClose(m_speed); 
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
