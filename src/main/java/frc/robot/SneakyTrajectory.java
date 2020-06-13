/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
    public Trajectory[] CenterRight6Cell = new Trajectory[2]; //yazdığımız otonomları burada tanımlıyoruz.
    public Trajectory[] Left3Cell = new Trajectory[1];
    public Trajectory[] Middle3Cell = new Trajectory[1];
    public Trajectory[] Right6Cell = new Trajectory[2];
    public Trajectory[] Left8Cell = new Trajectory[4];
    public Trajectory[] Right3Cell = new Trajectory[1];
    private DriveSubsystem m_drive;


    

    public SneakyTrajectory(DriveSubsystem drive){
       m_drive =drive;
        
       var autoVoltageConstraint =
      
       new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
         DriveConstants.kDriveKinematics,
         10);
 
         TrajectoryConfig configForward =
         new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                              DriveConstants.kMaxAccelerationMetersPerSecondSquared)
             // Add kinematics to ensure max speed is actually obeyed
             .setKinematics(DriveConstants.kDriveKinematics)
             // Apply the voltage constraint
             .addConstraint(autoVoltageConstraint);
 
            
             TrajectoryConfig configBackward =
           new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                              DriveConstants.kMaxAccelerationMetersPerSecondSquared)
             // Add kinematics to ensure max speed is actually obeyed
             .setKinematics(DriveConstants.kDriveKinematics)
             // Apply the voltage constraint
             .addConstraint(autoVoltageConstraint);
 
             configBackward.setReversed(true);

                    CenterRight6Cell[0] = TrajectoryGenerator.generateTrajectory( //ilk aşama olduğu için 0 yazdık.
                    List.of(
                     new Pose2d(12.80, 5.79, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                     new Pose2d(9.75, 7.54, new Rotation2d(0)),
                     new Pose2d(7.92, 7.54, new Rotation2d(0))),
                     configBackward);

              
                     CenterRight6Cell[1] = TrajectoryGenerator.generateTrajectory(  // ikinci aşama olduğu için 1 yazık.
                     List.of(
                     new Pose2d(7.92, 7.54, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                     new Pose2d(10.97, 5.79, new Rotation2d(0)),
                     new Pose2d(12.80, 5.79, new Rotation2d(0))),
                      configForward);    
     
 
                      Middle3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(3.10, 2.40, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                      new Pose2d(1.40, 2.42, new Rotation2d(3.141592654))),
                     configForward);    
 
 
 
                     Left3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(3.11, 4.42, new Rotation2d(-2.539305709)), //rotation açı fakat radyan biriminden.
                      new Pose2d(1.56, 3.34, new Rotation2d(-2.539305709))),
                     configForward);    
             
 
                     
                    
                    
                     Right6Cell[0] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(12.81, 5.81, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                      new Pose2d(10.77, 7.53, new Rotation2d(3.141592654)),
                      new Pose2d(8.08, 7.53, new Rotation2d(3.141592654))),
                     configBackward);  
 
 
                     Right6Cell[1] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(8.08, 7.53, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                      new Pose2d(10.77, 7.53, new Rotation2d(0)),
                      new Pose2d(12.81, 5.81, new Rotation2d(0))),
                     configForward);  
 
 
 
 
 
 
                     Left8Cell[0] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(12.78, 0.80, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                      new Pose2d(10.00, 0.80, new Rotation2d(3.141592654))),
                     configBackward);  
 
 
                     Left8Cell[1] = TrajectoryGenerator.generateTrajectory(  
                       List.of(
                       new Pose2d(10.00, 0.80, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                       new Pose2d(12.92, 4.12, new Rotation2d(0.499347426))),
                      configForward);  
 
 
                      Left8Cell[2] = TrajectoryGenerator.generateTrajectory(  
                       List.of(
                       new Pose2d(12.92, 4.13, new Rotation2d(0.499347426)), //rotation açı fakat radyan biriminden.
                       new Pose2d(11.36, 7.29, new Rotation2d(2.413167273)),
                       new Pose2d(8.10, 7.53, new Rotation2d(-3.114424858))),
                      configBackward);  
 
                      
                      Left8Cell[3] = TrajectoryGenerator.generateTrajectory(  
                       List.of(
                       new Pose2d(8.10, 7.53, new Rotation2d(-3.114424858)), //rotation açı fakat radyan biriminden.
                       new Pose2d(10.81, 7.43, new Rotation2d(-0.205395582)),
                       new Pose2d(12.78, 5.78, new Rotation2d(0))),
                      configForward);  
 
 
 
 
 
                      Right3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                       List.of(
                       new Pose2d(12.81, 7.78, new Rotation2d(-0.550386089)), //rotation açı fakat radyan biriminden.
                       new Pose2d(14.25, 7.06, new Rotation2d(-0.550386089))),
                      configForward);  
 
             


                   

     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
                    }

     public RamseteCommand getRamsete(Trajectory trajectory){
        
        return new RamseteCommand(
        trajectory,
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
        

     }
    
    }