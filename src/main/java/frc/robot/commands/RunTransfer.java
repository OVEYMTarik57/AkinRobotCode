/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;

public class RunTransfer extends CommandBase {
  private final TransferSubsystem m_transfer;
  private final double m_speed;
  
  public RunTransfer(TransferSubsystem transfer, double speed) {
    m_transfer = transfer;
    m_speed = speed;
    addRequirements(m_transfer);
    
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    m_transfer.runTransfer(m_speed);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopTransfer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
