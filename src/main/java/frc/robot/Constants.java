package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.RevPIDGains;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 20;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(25); //TODO: This must be tuned to specific robot
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
        public static final int angleContinuousCurrentLimit = 28; //25
        public static final int anglePeakCurrentLimit = 50;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 38; //35
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 8.0;//10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.53);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(258.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(212.60);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.72);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final double speedMod = 0.5;
        public static final double balanceSpeedMod = .47;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 4;
        public static final double kPYController = 4;
        public static final double kPXandYControllers = 5;
        public static final double kPThetaController = 5;

        public static final double maxPlatformPositivePitch = 6;
        public static final double maxPlatformNegativePitch = -6;

        public static final double desiredBalanceAngle = -1.6;
        public static final double balanceP = .5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ExtenderConstants {
        
        public static final int extenderMotorID = 13;
        public static final int extenderDistanceSensorID = 21;

             // PID coefficients
        public static final double extenderKP = 0.00; 
        public static final double extenderKI = 0.00000;
        public static final double extenderKD = 0; 
        public static final double extenderKIz = 0; 
        public static final double extenderKFF = 0.002; 
        public static final double extenderKMaxOutput = 1; 
        public static final double extenderKMinOutput = -1;
        public static final double extenderMaxRPM = 5700;

        // Smart Motion Coefficients
        public static final int extenderMinVel = 0; // rpm
        public static final int extenderMaxVel = 2000; // rpm
        public static final int extenderMaxAcc = 2000;
        public static final double extenderAllowedErr = .2;

        private final static double _gearRatio = 22; // 4:1 cartridge + 4:1 cartrdige + 1.375 
        private final static double _winchDia_in = 1.79; // For PWF Arm
        private final static double _winchDia_m = Units.inchesToMeters(_winchDia_in);
        private final static double _winchCircumference_m = _winchDia_m * Math.PI;
        public final static double KExtenderMetersToNeoRotationsFactor = _gearRatio / _winchCircumference_m;
       
         
    }
    
    public static final class ElevatorConstants {

        public static final int elevatorMotorID = 14;
        public static final int elevatorDistanceSensorID = 22;

        // PID coefficients
        public static final double elevatorKP = 0.0000; 
        public static final double elevatorKI = 0.00000;
        public static final double elevatorKD = 0; 
        public static final double elevatorKIz = 0; 
        public static final double elevatorKFF = 0.0018; 
        public static final double elevatorKMaxOutput = 1; 
        public static final double elevatorKMinOutput = -1;
        public static final double elevatorMaxRPM = 5700;

        // Smart Motion Coefficients
        public static final int elevatorMinVel = 0; // rpm
        public static final int elevatorMaxVel = 2000; // rpm
        public static final int elevatorMaxAcc = 2000;
        public static final double elevatorAllowedErr = .2;

        private final static double _gearRatio = 49.5; // 4:1 cartridge + 3:1 cartrdige + 3:1 Cartridge + 1.375 
        private final static double _winchDia_in = 1.79; // For PWF Arm
        private final static double _winchDia_m = Units.inchesToMeters(_winchDia_in);
        private final static double _winchCircumference_m = _winchDia_m * Math.PI;
        public final static double KElevatorMetersToNeoRotationsFactor = _gearRatio / _winchCircumference_m;
        
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 15;
    }

    public static final class WristConstants {
        
        public static final int wristMotorID = 16;

         // PID coefficients
         public static final double wristKP = 0.0000; 
         public static final double wristKI = 0.00000;
         public static final double wristKD = 0; 
         public static final double wristKIz = 0; 
         public static final double wristKFF = 0.0015; 
         public static final double wristKMaxOutput = 1; 
         public static final double wristKMinOutput = -1;
         public static final double wristMaxRPM = 5700;
 
         // Smart Motion Coefficients
         public static final int wristMinVel = 0; // rpm
         public static final int wristMaxVel = 2500; // rpm
         public static final int wristMaxAcc = 2000;
         public static final double wristAllowedErr = .2;
        

    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

}