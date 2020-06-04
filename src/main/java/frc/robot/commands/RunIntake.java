/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final double m_speed;
   
  public RunIntake(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(m_intake);
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    m_intake.runIntake(m_speed);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
