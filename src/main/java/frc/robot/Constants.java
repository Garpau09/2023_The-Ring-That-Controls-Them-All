package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 27;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23); 
        public static final double wheelBase = Units.inchesToMeters(23.5); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

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
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        
        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); 
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.0; 

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(241.26);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(83.14);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class RearLeft { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.37);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class RearRight { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.20);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18);
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
      }

    public static final class StartingConstants {
        public static final Pose2d aprilTag1 = new Pose2d(13.313558, 1.071626, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d aprilTag2 = new Pose2d(13.313558, 2.748026, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d aprilTag3 = new Pose2d(13.313558, 4.424426, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d aprilTag6 = new Pose2d(2.2, 4.424426, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d aprilTag7 = new Pose2d(2.2, 2.748026, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d aprilTag8 = new Pose2d(2.2, 1.071626, new Rotation2d(Math.toRadians(180)));

        public static final Pose2d[] startingPositions = new Pose2d[]{
            aprilTag1,
            aprilTag2,
            aprilTag3,
            null,
            null,
            aprilTag6,
            aprilTag7,
            aprilTag8
        };
        
    }
    


    public static final class ClawConstants{
        public static enum ClawPoses{
            LOADING,
            TRANSPORT,
            LOW_SCORE,
            MID_SCORE,
            HIGH_SCORE,
            TRANSITION,
            GRABBING
        }
     
        public static final HashMap<ClawPoses, double[]> kClawStates = new HashMap<ClawPoses, double[]>(){
            {
            /**
             * @param Wrist Position, Arm Position
             */
            put(ClawPoses.LOADING, new double[] {335, 0}); //TODO: Change these encoder values
            put(ClawPoses.TRANSPORT, new double[] {76, 0});
            put(ClawPoses.LOW_SCORE, new double[] {151, 11});
            put(ClawPoses.MID_SCORE, new double[] {170, 75});
            put(ClawPoses.HIGH_SCORE, new double[] {203, 100});
            put(ClawPoses.TRANSITION, new double[] {0, 50}); //Transition is not used on the wrist
            put(ClawPoses.GRABBING, new double[] {345, 0});
            }
        };

        public static final double armGearRatio = 144;
        public static final double wristGearRatio = 1;

        public static final double armFeedForward = 0.09;

        public static final int ClawMotorWristID = 28;
        public static final int ClawMotorLeftID = 26;
        public static final int ClawMotorRightID = 25;
        public static final int GripperEncoder = 29;
        public static final int wristEncoder = 30;

        public static final double wristGrabbed = 345; // Values Changed 2/26 2:53PM WAS 326
        public static final double wristLoading = 335; 
        public static final double wristTransport = 76;
        public static final double wristLow = 151;
        public static final double wristMedium = 170;
        public static final double wristHigh = 203;

        public static final double armLoading = 0;
        public static final double armTransport = 0;
        public static final double armLow = 11;
        public static final double armMedium = 75;
        public static final double armHigh = 100;
        public static final double armTransition = 50;

    }
}
