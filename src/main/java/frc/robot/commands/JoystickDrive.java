/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public JoystickDrive(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier rotation) {
    m_drive = drive;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_drive);
    }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    m_drive.ArcadeDrive(m_forward.getAsDouble(),m_rotation.getAsDouble());
  }

  
  @Override
  public void end(boolean interrupted) {
  }

 
  @Override
  public boolean isFinished() {
    return false;
  }
}
