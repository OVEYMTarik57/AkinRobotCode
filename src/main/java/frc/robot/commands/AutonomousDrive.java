/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDrive extends CommandBase {
  /**
   * Creates a new AutonomousDrive.
   */
    private final DriveSubsystem m_drive;
    private final double m_speed;
    private final double m_distance;
    private final double m_turn;

  
   public AutonomousDrive(final DriveSubsystem drive, final double speed, final double distance, final double turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = drive;
    this.m_speed = speed;
    this.m_distance =distance;
    this.m_turn = turn;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, m_speed); // rot a dönüş için m_speed vermemiz gerek.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
      return 
      m_drive.getRightWheelCm()>=m_distance &&
      m_drive.getHeading()>=m_turn;
  
    
  }
}
