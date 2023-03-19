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
        public static final double trackWidth = Units.inchesToMeters(19.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(28.75); //TODO: This must be tuned to specific robot
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
        public static final double kPXandYControllers = 4;
        public static final double kPThetaController = 4;

        public static final double maxPlatformPositivePitch = 6.6;
        public static final double maxPlatformNegativePitch = -8.6;

        public static final double desiredBalanceAngle = -1.6;
        public static final double balanceP = .5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ExtenderConstants {
        
        public static final int extenderMotorID = 13;
        public static final int extenderDistanceSensorID = 20;

         // The following FeedForward estimates are theoritical values calculated from recalc tool (reca.lc)
        // https://www.reca.lc/linear?efficiency=80&load=%7B%22s%22%3A120%2C%22u%22%3A%22lbs%22%7D&
        // motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A20%2C
        // %22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A0.787%2C%22u%22%3A%22in
        // %22%7D&travelDistance=%7B%22s%22%3A24%2C%22u%22%3A%22in%22%7D
        // Note: Assuming 0.5 power is sufficient to move the Elevator up. 0.5 power = 12*0.5 = 6Volts
        public final static double kGVolts = 0.06; // Volts
        public final static double kSVolts = .6-kGVolts; // armkSVolts = Volts_that_take_to_move_the_arm_from_rest - 
                                                                //               armkSVolts
        public final static double kVVoltSecondPerMeter = 30.86; // Volts * sec / radians
        public final static double kAVoltSecondSquaredPerMeter = 0.01; // Volts * sec^2 / radians

        // The following Feedback estimates are taken from the WPILIB's ArmbotOffboard example
        // These are not really applicable to ClimberElevator; they have to be tuned
        // based on experimentation
        // PID values for SparkMaxPIDController
        public final static double kP = 1;
        public final static double kI = 0;
        public final static double kD = 0;
        // Min and Max output power allowed by SparkMaxPIDController
        public final static double kMinOutput = -1;
        public final static double kMaxOutput = 1;

        // Elevator Position
        // Convention: _in=inches, _m=meters
        //HOME, FLOOR_CUBE, FLOOR_CONE, SHELF, MID, HIGH
        private final static double _homePos_in = 0;
        private final static double _floorCubePos_in = 8;
        private final static double _floorConePos_in = 12;
        private final static double _shelfPos_in = 15;
        private final static double _midPos_in = 17;
        private final static double _highPos_in = 20;        
        
        public final static double kHomePos_m = Units.inchesToMeters(_homePos_in);
        public final static double kFloorCubePos_m = Units.inchesToMeters(_floorCubePos_in);        
        public final static double kFloorConePos_m = Units.inchesToMeters(_floorConePos_in);
        public final static double kShelfPos_m = Units.inchesToMeters(_shelfPos_in);       
        public final static double kMidPos_m = Units.inchesToMeters(_midPos_in);
        public final static double kHighPos_m = Units.inchesToMeters(_highPos_in);


        private final static double _gearRatio = 36; // 4:1 cartridge + 3:1 cartrdige + 3:1 Cartridge 
        private final static double _winchDia_in = 1.79; // For PWF Arm
        private final static double _winchDia_m = Units.inchesToMeters(_winchDia_in);
        private final static double _winchCircumference_m = _winchDia_m * Math.PI;
        public final static double KExtenderMetersToNeoRotationsFactor = _gearRatio / _winchCircumference_m;

        // The following values in terms of Neo motor shaft rotations
        private final static double _homePosNeoRotations = KExtenderMetersToNeoRotationsFactor * kHomePos_m;
        private final static double _floorCubesPosNeoRotations = KExtenderMetersToNeoRotationsFactor * kFloorCubePos_m;
        private final static double _floorConesPosNeoRotations = KExtenderMetersToNeoRotationsFactor * kFloorConePos_m;
        private final static double _shelfPosNeoRotations = KExtenderMetersToNeoRotationsFactor * kShelfPos_m;
        private final static double _midPosNeoRotations = KExtenderMetersToNeoRotationsFactor * kMidPos_m;
        private final static double _highPosNeoRotations = KExtenderMetersToNeoRotationsFactor * kHighPos_m;

        // Elevator position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)_highPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)_homePosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        
        // The following are constrains for the TrapezoidalProfile
        // Note that the TrapezoidalProfile takes values in Meters whereas SparkMax's PIDController
        // use number of Shaft Rotations
        private final static double _maxVelElevaor_in_p_s = 10; // => 10 inches-per-sec velocity
        private final static double _secondsToPeakVel = 1; // => 1 second to 0-to-peak velocity
        private final static double _maxAccelElevator_in_p_ss = _maxVelElevaor_in_p_s / _secondsToPeakVel;
        public final static double kMaxVelMetersPerSec = Units.inchesToMeters(_maxVelElevaor_in_p_s);
        public final static double kMaxAccelMetersPerSecSquared = Units.inchesToMeters(_maxAccelElevator_in_p_ss);
        public final static double kInitialPosMeters = Units.inchesToMeters(_homePos_in);
        
    }
    
    public static final class ElevatorConstants {

        public static final int elevatorMotorID = 14;
        public static final int extenderDistanceSensorID = 21;
        
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 15;
    }

    public static final class WristConstants {
        
        public static final int wristMotorID = 16;

    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

}