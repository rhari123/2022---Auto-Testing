// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Limelight {
        public static final double kTargetHeight = 36;
        public static final double kLimeLightHeight = 28.25;
        public static final double kLimelightAngle = 21;//degrees
    }

    public static final class ChassisC {
        public static final double kLowGearRatio = 0;  //need to change these
        public static final double kHighGearRatio = 0;  

        public static final double PPR = 2048;  

        public static final double wheelRadius = 3;  //need to change these
        public static final double wheelRadiusMeters = wheelRadius/39.37;  //For Autos

        public static final double maxVelocity = 20;  //need to change these
        public static final double maxAcceleration = 100;  
        public static final double maxDecceleration = 25; 

        public static final double trackWidth = 0;  //need to change these

        public static final double kF = 0.0682;
        public static final double kP = 0.00125;
    }

    public static final class Joystick { 
        public static final int leftStick = 0;
        public static final int rightStick = 1;
        public static final int xboxController = 2;

        public static final int gearShiftButton = 7;
        public static final int toggleServo = 9; 

        public static final int turnButton = 8; // temp for testing turn 180
        public static final int intakeOutButton = 10;
    }

    public static final class IntakeC {

    }


    public static final class ServoC {
        public static final double lowAngle = 55; //need to change these
        public static final double mediumAngle = 90;
        public static final double highAngle = 180;
    }

    public static final class ShooterC {
        public static final int wheelRadius = 2;
        public static final double PPR = 2048;
        public static final double kF = 0.0682;
        public static final double kP = 0.00125;
        public static final double stallTime = 5;
        public static boolean isReady = false;
    }

    public static final class HorizontalAngler {
        public static final double PPR = 2048;
        public static final double kF = 0.0682;
        public static final double kP = 0.00125;
        public static final double radius = 10;
        public static final double kRightOffset = 5;
        public static final double kLeftOffset = 5;
    }

    public static final class VerticalAngler {
        public static final double PPR = 4096;
        public static final double anglerPIDkP = 0.01;
        public static final double radius = 1;
    }

    public static final class DIO {
        public static final int kShooterLeftLimit = 2;
        public static final int kShooterRightLimit = 3;

        public static final int kVerticalLimit = 1; 

        public static final int kTowerIn = 4; 
        public static final int kTowerOut = 0;

        public static final int kLeftMin = 0; //Proxes for climber
        public static final int kLeftMax = 0;
        public static final int kRightMin = 0;
        public static final int kRightMax = 0;
    }

    public static final class CANID {
        public static final int kPCM = 14;

        public static final int kLeftMaster = 2;
        public static final int kLeftSlaveA = 1;
        public static final int kLeftSlaveB = 3;

        public static final int kRightMaster = 5;
        public static final int kRightSlaveA = 4;
        public static final int kRightSlaveB = 6;

        public static final int kIntake = 7;

        public static final int kIndexerTower = 8; 

        public static final int kHorizontalAngler = 10; 
        public static final int kVerticalAngler = 11;

        public static final int kShooter1 = 12;
        public static final int kShooter2 = 13;

         //CAN ID 14 is PCM (at the top)

        public static final int kHorizontalCANCoder = 16;
        public static final int kVerticalCANCoder = 17;
       
        public static final int kClimberExtenderM = 18; //right 
        public static final int kClimberExtenderS = 19; //left 
    }

    public static final class PCM {
        public static final int kLeftGearForward = 1;
        public static final int kLeftGearReverse = 0;

        public static final int kRightGearForward = 3;
        public static final int kRightGearReverse = 2;

        public static final int kRightIntakeLifterForward = 5;
        public static final int kRightIntakeLifterReverse = 4;

        public static final int kLeftIntakeLifterForward = 7;
        public static final int kLeftIntakeLifterReverse = 6;

    }

    public static final class ClimberC {
        // public static final double kF = 0.00976;
        public static final double kP = 0.0001;
        public static final double kD = 0.01;
        //public static final double extenderLimit = 0;
        public static final double lowerLimit = 10;
        public static final double upperLimit = 50000;
    }

    public static final class Autos {
        public static final double maxSpeed = 3; 
        public static final double maxAcceleration = 3; 
        
        public static final double kRamseteB = 2; //Meters
        public static final double kRamseteZeta = 0.7; //Meters

        public static final double kS = 0.72126;
        public static final double kV = 2.55;
        public static final double kA = 0.38667;
        public static final double kP = 8.5; //NEED TO FIND RIGHT VALUE
        
        public static final double kTrackWidth = 24; //NEED TO FIND RIGHT VALUE
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
        public static Trajectory trajectory;

    }
}
