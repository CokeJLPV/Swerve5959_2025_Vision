package com.team5959;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//  =================
//       Red CAN IDS Reference
//  =================
//   0 · roboRIO
//   1 · PDH
//   2 · frontLeftRotation
//   3 · frontLeftDrive
//   4 · frontRightRotation
//   5 · frontRightDrive
//   6 · rearRightRotation
//   7 · rearRightDrive
//   8 · rearLeftRotation
//   9 · rearLeftDrive
//  =================
//  10 . frontLeftAbsEncoder
//  11 . frontRightAbsEncoder
//  12 . rearRightAbsEncoder
//  13 . rearLeftAbsEncoder

public class Constants {
    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
      }
    
      public static class SwerveConstants {
        //SDS L2 
        public static final boolean ROTATION_ENCODER_DIRECTION = false; 
    
        /* * * MEASUREMENTS * * */
        public static final double WHEEL_DIAMETER = 4 * 2.54 / 100; //Diametro en metros
        public static final double TRACK_WIDTH = 0.6000;
        public static final double WHEEL_BASE = 0.6000;
      
        public static final double DRIVE_GEAR_RATIO = 6.75 / 1;
        public static final double ROTATION_GEAR_RATIO = 150 / 7;
        
        public static final double VOLTAGE = 12;
    
        /* * * SWERVE DRIVE KINEMATICS * * */
        // ORDER IS ALWAYS FL, BL, FR, BR 
        //pos x is positive out in front, pos y is positive to the left 
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
          
          // front left
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
          // back left
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
          // front right
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
          // back right
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)                 
    
        );
    
        /* * * FRONT LEFT * * */
        
        public static class FrontLeft {
          public static final int DRIVE_PORT = 3;
          public static final int ROTATION_PORT = 2;
          public static final int ABSOLUTE_ENCODER_PORT = 10;
          public static final double OFFSET = (0.39209); //80.95; (-0.3986 * 90)//este ya está bien-143.87
          public static final boolean DRIVE_INVERTED = false; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
    
        /* * * BACK LEFT * * */
       
        public static class BackLeft {
          public static final int DRIVE_PORT = 9;
          public static final int ROTATION_PORT = 8;
          public static final int ABSOLUTE_ENCODER_PORT = 13;
          public static final double OFFSET = (0.10351); //(-0.0927 * 90)-101.60 + 6;
          public static final boolean DRIVE_INVERTED = false; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }

        /* * * FRONT RIGHT * * */
          public static class FrontRight {
          public static final int DRIVE_PORT = 5;
          public static final int ROTATION_PORT = 4;
          public static final int ABSOLUTE_ENCODER_PORT = 11;
          public static final double OFFSET = (-0.43774); //-25.31 - 2;(0.4321 * 55)
          public static final boolean DRIVE_INVERTED = true; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
    
        /* * * BACK RIGHT * * */
        // FILL IN VALUES FOR BACK RIGHT 
        public static class BackRight {
          public static final int DRIVE_PORT = 7;
          public static final int ROTATION_PORT = 6;
          public static final int ABSOLUTE_ENCODER_PORT = 12;
          public static final double OFFSET = (0.20160); //(-0.2290 * )-28.92 + 6;
          public static final boolean DRIVE_INVERTED = true; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
        
    
        
        
        /* * * CONVERSIONS FOR ENCODERS * * */
        //velocity in meters per sec instead of RPM 
        public static final double DRIVE_ENCODER_POSITION_CONVERSION = ((2 * Math.PI * (WHEEL_DIAMETER/2))) / DRIVE_GEAR_RATIO; //drive enc rotation
        //velocity in meters instead of rotations 
        public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed por segundo.
     
        /* * * PID VALUES FOR TURNING MOTOR PID * * */
        public static final double KP_TURNING = 0.006;
        public static final double KI_TURNING = 0.0002;
        public static final double KD_TURNING = 0.0001;

        public static final double DRIVE_KP = 0.05;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0001;
    
        public static final double KP_AUTO_XController = 2.9;
        public static final double KI_AUTO_XController = 0.0005;
        public static final double KD_AUTO_XController = 0.001;
        public static final double AUTO_XTOLLERANCE = 0.025; // tolerance in meters

        public static final double KP_AUTO_YController = 2.85;
        public static final double KI_AUTO_YController = 0.0005;
        public static final double KD_AUTO_YController = 0.001;
        public static final double AUTO_YTOLLERANCE = 0.025; // tolerance in meters
    
        public static final double KP_AUTO_ROTATION = 1.1;
        public static final double KI_AUTO_ROTATION = 0.0005;
        public static final double KD_AUTO_ROTATION = 0.001;
        public static final double ROTATION_TOLLERANCE = 1; // tolerance in dergrees

        public static final double KP_AUTO_HOLDING = 0.15;
        public static final double KI_AUTO_HOLDING = 0.000;
        public static final double KD_AUTO_HOLDING = 0.001;
        public static final double HOLDING_TOLLERANCE = 0.1; // tolerance in dergrees

        public static final double KP_TRANS_PATHPLANNER =5;
        public static final double KI_TRANS_PATHPLANNER = 0.0001;
        public static final double KD_TRANS_PATHPLANNER = 0.001;
        public static final double PIDPATHPLANNER_TRANS_TOLLERANCE = 1; // tolerance in dergrees

        public static final double KP_ROT_PATHPLANNER = 5;
        public static final double KI_ROT_PATHPLANNER = 0.001;
        public static final double KD_ROT_PATHPLANNER = 0.0001;
        public static final double PIDPATHPLANNER_ROT_TOLLERANCE = 1; // tolerance in dergrees
    
    
        /* * * MAX * * */
        public static final double MAX_SPEED = 3.45; //meters per second
        public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);


        //AUTONOMOUS CONSTANTS
        public static final double MAX_AUTO_SPEED = 2; //meters per second
        public static final double MAX_AUTO_ACCELERATION = 1.5; //meters per second squared

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                  Math.toRadians(180), Math.toRadians(360));

        
      }
      //INTAKE CONSTANTS
      public static class intakeCoralConstants{ 
        //ID's
        public static final int coralIntakeMotorRightID = 20;
        public static final int coralIntakeMotorLeftID = 21;
        //INVERTED
        public static final boolean coralIntakeMotorRightInverted = false;
        public static final boolean coralIntakeMotorLeftInverted = true;

      }

      public static class intakeAlgaeConstants{ 
        //ID's
        public static final int armMotorID = 18;
        public static final int absoluteEncoderPort = 3;
        public static final int algaeIntakeMotorID = 19;

        //INVERTED
        public static final boolean algaeIntakeMotorInverted = false;

        //PID VALUES
        public static final double KP_ARM = 0.009;
        public static final double KI_ARM = 0.00; //FIXME adjust pid values for arm
        public static final double KD_ARM = 0.0001;

        //POSITION VALUES (in encoder units)
        public static final int armIntakeInStartingPosition = 100;
        public static final int armIntakeInPerimeterPosition = 80;
        public static final int armIntakeOutPosition = 145;
      }

      //ELEVATOR CONSTANTS
      public static class ElevatorConstants{
        public static final int elevatorRightID = 16;
        public static final int elevatorLeftID= 17;
        public static final boolean elevatorRightInverted = false;
        public static final boolean elevatorLeftInverted = true;
          
        //PID VALUES
        public static final double KP_ELEVATOR = 0.03;
        public static final double KI_ELEVATOR = 0.000; //FIXME adjust pid values for elevator
        public static final double KD_ELEVATOR = 0.0001;
        public static final double KP_SP_ELEVATOR = 0.003;  //SPECIAL P FOR STARTING POSITION
        //POSITION VALUES (in encoder units)
        public static final double elevatorStartingPosition = -13.00;
        public static final double elevatorL1Position = -60.00;  //-60
        public static final double elevatorL2Position = -93.00; //-93 //USE NEGATIVES
        public static final double elevatorL3Position = -190.00; //-190
  
        //SOFT LIMITS
        public static final double elevatorUpperLimit = -190.00;
        public static final double elevatorLowerLimit = -13.00;
        public static final boolean forwardSoftLimitEnabled = true;
        public static final boolean reverseSoftLimitEnabled = true;
      }

      public static class miniArmConstants{
        //ID's
        public static final int miniArmMotorID = 22;
        public static final int absoluteEncoderPort = 6; //FIXME change to correct port
        //PID VALUES
        public static final double KP_MINI_ARM = 0.005;
        public static final double KI_MINI_ARM = 0.000; //FIXME adjust pid values for mini arm
        public static final double KD_MINI_ARM = 0.0001;
        //POSITION VALUES (in encoder units)
        public static final double miniArmStartingPosition = 150.00;
        public static final double miniArmDropAlgaePosition = 110.00;
        public static final double miniArmDownPosition = 80;
  
      }
    }
    

