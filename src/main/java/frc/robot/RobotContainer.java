/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.LockClimb;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransfer;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.autonomous.CenterRight6Cell;
import frc.robot.commands.autonomous.Left3Cell;
import frc.robot.commands.autonomous.Left8Cell;
import frc.robot.commands.autonomous.Middle3Cell;
import frc.robot.commands.autonomous.Right3Cell;
import frc.robot.commands.autonomous.Right6Cell;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final SneakyTrajectory m_SneakyTrajectory = new SneakyTrajectory(m_drive);



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
    new JoystickButton(m_driverController, 2).whileHeld(new RunShooter(m_shooter, 0.6));
    //
    new JoystickButton(m_driverController, 3).whileHeld(new RunIntake(m_intake, 0.8));
    new JoystickButton(m_driverController, 4).whileHeld(new RunIntake(m_intake, -0.4));
    new JoystickButton(m_driverController, 5).toggleWhenPressed(new ToggleIntake(m_intake, 0.9));
    new JoystickButton(m_driverController, 6).whileHeld(new RunShooter(m_shooter, 0.6));
    new JoystickButton(m_driverController, 7).whileHeld(new RunShooter(m_shooter, -0.32));
    new JoystickButton(m_driverController, 8).whileHeld(new RunClimb(m_climb, 0.5));
    new JoystickButton(m_driverController, 9).whileHeld(new RunClimb(m_climb, -0.5));
    new JoystickButton(m_driverController, 10).toggleWhenPressed(new LockClimb(m_lock, 0.9));
    new JoystickButton(m_driverController, 11).toggleWhenPressed(new RunTransfer(m_transfer, 0.3));
    new JoystickButton(m_driverController, 12).toggleWhenPressed(new RunTransfer(m_transfer, -0.3));
    

    
    
  }


  public Command trajectoryCommand(){
      
    // Create a voltage constraint to ensure we don't accelerate too fast
  var autoVoltageConstraint =
    
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);

      TrajectoryConfig config =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

         
          TrajectoryConfig configReversed =
        new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

          configReversed.setReversed(true);


          
          Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
  );

  RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    m_drive::getPose,
    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    m_drive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_drive::tankDriveVolts,
    m_drive
);

    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));

}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer auto) {
    // An ExampleCommand will run in autonomous
    
    switch(auto){
      case 1:
      return new Left8Cell(m_SneakyTrajectory, m_shooter, m_intake, m_hopper, m_drive, m_transfer);
      case 2:
      return new Right3Cell(m_SneakyTrajectory, m_shooter, m_intake, m_hopper, m_drive, m_transfer);
      case 3:
      return new CenterRight6Cell(m_SneakyTrajectory, m_shooter, m_intake, m_hopper, m_drive,m_transfer);
      case 4:
      return new Left3Cell(m_SneakyTrajectory, m_shooter, m_intake, m_hopper, m_drive, m_transfer);
      case 5: 
      return new Middle3Cell(m_SneakyTrajectory, m_shooter, m_intake, m_hopper, m_drive, m_transfer);
     
      default:
      return null;


  }
  }
}
