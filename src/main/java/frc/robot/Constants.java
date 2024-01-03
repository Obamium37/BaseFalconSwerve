package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import frc.lib.util.COTSFalconSwerveConstants; Possibly add some values to SwerveX
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
       // public static final int pigeonID = 1; 


        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-// Done




        //public static final COTSFalconSwerveConstants chosenModule =  
            //COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);



        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.25); // Done, set value
        public static final double wheelBase = Units.inchesToMeters(23.25); // Done, Set value
        public static final double wheelCircumference = 12.56; // Done Set value



        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         // No need to change
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));



        // Most of these should be presets found in the COTS module

        /* Module Gear Ratios */
        public static final double driveGearRatio = 9.63;//(6.55/1.0); // Done Set value
        public static final double angleGearRatio = 15.43;//(10.29/1.0); // Done Set value



        /* Motor Inverts */
        public static final boolean angleMotorInvert = true; // Done, set
        public static final boolean driveMotorInvert = false; // Done, set



        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false; // Done, set



        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;



        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;



        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;



        /* Angle Motor PID Values */
        public static final double angleKP = 0.3; // Done, set value
        public static final double angleKI = 0.0; // Done, set value
        public static final double angleKD = 0.0; // Done, set value
        public static final double angleKF = 0.0; // Done, set value



        /* Drive Motor PID Values */
        public static final double driveKP = 0.0; 
        public static final double driveKI = 0.0; // Leave rest at 0.0
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; 

        /* Drive Motor Characterization Values -
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.23627 / 12); //TODO: This must be tuned to specific robot // 0.23
        public static final double driveKV = (1.4301 / 12); // 0.05
        public static final double driveKA = (0.30074/ 12); // 0.01



        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // Done, Set value



        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //Done, set value




        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;




        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class lfConsts { 
            public static final int driveMotorID = 3; // Done, from CANID_LEFTFRONT_DRIVE  // 3


            public static final int angleMotorID = 13; // 13, from CANID_LEFTFRONT_STEER

            
            public static final int canCoderID = 23;  // 7
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }



        /* Front Right Module - Module 1 */
        public static final class rfConsts { 
            public static final int driveMotorID = 4;  //Done, from CANID_RIGHTFRONT_DRIVE  // 4


            public static final int angleMotorID = 14; // 14 from CANID_LEFTFRONT_STEER


            public static final int canCoderID = 24;  // 8
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        
        /* Back Left Module - Module 2 */
        public static final class lbConsts {

            public static final int driveMotorID = 5;  //Done, from CANID_LEFTBACK_DRIVE  // 5

            public static final int angleMotorID = 15; //15, from CANID_LEFTFRONT_STEER

            public static final int canCoderID = 21;  // 9

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class rbConsts { 

            public static final int driveMotorID = 6; //Done, from CANID_RIGHTBACK_DRIVE // 6

            public static final int angleMotorID = 16; // 16, from CANID_LEFTFRONT_STEER

            public static final int canCoderID = 22;  //10

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); 

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }


    

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
        public static final double kPXController = 0.0;
        public static final double kPYController = 0.0;
        public static final double  kPThetaController = 0.0;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
