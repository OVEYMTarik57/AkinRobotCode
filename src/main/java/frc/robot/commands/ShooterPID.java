/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterPID extends PIDCommand {
  
  private ShooterSubsystem m_shooter;
  private static double m_motorOutput;
  private final static SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(ShooterConstants.kS,
  ShooterConstants.kV, ShooterConstants.kA);
  private boolean isInterruptable;

  public ShooterPID(double targetRPM, ShooterSubsystem shooter) {
    super(
        
        new PIDController(ShooterConstants.turnP, ShooterConstants.turnI, ShooterConstants.turnD),
        //
        () -> shooter.getRPM(),
        //
        targetRPM,
        //
        output -> {
        //  
        m_motorOutput = output + m_shooterFeedForward.calculate(targetRPM);
        shooter.runShooterVoltage(m_motorOutput);
      });
      getController().setTolerance(100);
    m_shooter = shooter;
    addRequirements(m_shooter);  
    
  }

  @Override
  public void initialize() {
    super.initialize();
    m_motorOutput = 0;
  }

  @Override
  public void execute() {
    super.execute();
    m_shooter.isAtSetpoint = getController().atSetpoint();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isInterruptable && getController().atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (!isInterruptable) {
      m_shooter.runShooterVoltage(0);
      m_shooter.isAtSetpoint = false;

}
}
}


