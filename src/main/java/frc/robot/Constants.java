/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class JoystickConstants{
        public static final int driverControllerPin = 0;
   
       }


    public static class DriveConstants{
        public static final int frontLeftMotorPin = 1;
        public static final int rearLeftMotorPin = 2;
        public static final int frontRightMotorPin = 3;
        public static final int rearRightMotorPin = 4; 

        public static final double turnP = 1.0;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;

        public static final double turnAccuracy = 2.0;
        public static final double distanceAccuracy = 3.0;


    }

    public static class IntakeConstants{
        public static final int intakeMotorPin = 5;
        public static final int intakeOpen1MotorPin = 6;
        public static final int intakeOpen2MotorPin = 7;

        public static final int intakeHallEffect = 1;
    }

    public static class HopperConstants{
        public static final int hopperBagMotorPin = 8;
        public static final int centerMotorPin = 9;
        public static final int topMotorPin = 10;
        public static final int bottomMotorPin = 11;

    }

    public static class ShooterConstants{
        public static final int shooterMotor1Pin = 12;
        public static final int shooterMotor2Pin = 13;
    
    }

    public static class ClimbConstants{
        public static final int climbMotorPin = 14;
        public static final int servoMotorPin = 15;
        
    }
}