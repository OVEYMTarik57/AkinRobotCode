/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.VisionLED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionLED;

public class CloseLED extends CommandBase {
  /**
   * Creates a new CloseLED.
   */
  private final VisionLED m_led;

  public CloseLED(VisionLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led;
    addRequirements(m_led);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.m_relay.set(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.m_relay.set(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.m_relay.set(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
