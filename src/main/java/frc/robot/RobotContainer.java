/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.LockClimb;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransfer;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final TransferSubsystem m_transfer = new TransferSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final ClimbSubsystem m_lock = new ClimbSubsystem();



  public Joystick m_driverController = new Joystick(JoystickConstants.driverControllerPin);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, 1).whileHeld(new RunIntake(m_intake, 0.8));
    new JoystickButton(m_driverController, 1).whileHeld(new RunHopper(m_hopper, 0.5));
    //
    new JoystickButton(m_driverController, 2).whileHeld(new RunHopper(m_hopper, 0.5));
    new JoystickButton(m_driverController, 2).whileHeld(new RunTransfer(m_transfer, 0.8));
    //
    new JoystickButton(m_driverController, 3).whileHeld(new RunIntake(m_intake, -0.4));
    new JoystickButton(m_driverController, 4).toggleWhenPressed(new ToggleIntake(m_intake, 0.9));
    new JoystickButton(m_driverController, 5).whileHeld(new RunShooter(m_shooter, 0.6));
    new JoystickButton(m_driverController, 6).whileHeld(new RunShooter(m_shooter, -0.32));
    new JoystickButton(m_driverController, 7).whileHeld(new RunClimb(m_climb, 0.5));
    new JoystickButton(m_driverController, 8).whileHeld(new RunClimb(m_climb, -0.5));
    new JoystickButton(m_driverController, 4).toggleWhenPressed(new LockClimb(m_lock, 0.9));
    

    
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
