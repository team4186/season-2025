package frc.robot.swervetesting
import kotlin.math.PI
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.util.Units

//class Constants {
    /* THIS IS THE JAVA VERSION
        public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kTurningRPM2RadPerSec = kDirveEncoderRot2Meter / 60;
        public static final double kPTurning = 0.5;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveMotorGearRatio =  1 / 5.8462;
        public static final double kDriveEncoderRPM2MeterPerSec = kTurningEncoderRot2Rad / 60;
    } */

class ModuleConstants {
    companion object {
        val kWheelDiameterMeters: Double = /*inchesToMeters(4)*/ 0.0254 * 4
        val kDriveMotorGearRatio: Double = 1 / 5.8462
        val kTurningMotorGearRatio: Double =  1 / 18.0
        val kDriveEncoderRot2Meter: Double = kDriveMotorGearRatio * PI * kWheelDiameterMeters
        val kTurningEncoderRot2Rad: Double = kTurningMotorGearRatio * PI * kWheelDiameterMeters
        val kDriveEncoderRPM2MeterPerSec: Double =  kDriveEncoderRot2Meter / 60
        val kTurningRPM2RadPerSec: Double = kTurningEncoderRot2Rad / 60
        val kPTurning: Double = 0.5
    }

    // these are not the actual values, change later
    object DriveConstants {
        const val kTrackWidth = Units.inchesToMeters(21.0) // Distance between right and left wheels
        const val kWheelBase = Units.inchesToMeters(25.5) // Distance between front and back wheels

        val kDriveKinematics = SwerveDriveKinematics(
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        )

        // Motor ports
        const val kFrontLeftDriveMotorPort = 8
        const val kBackLeftDriveMotorPort = 2
        const val kFrontRightDriveMotorPort = 6
        const val kBackRightDriveMotorPort = 4

        const val kFrontLeftTurningMotorPort = 7
        const val kBackLeftTurningMotorPort = 1
        const val kFrontRightTurningMotorPort = 5
        const val kBackRightTurningMotorPort = 3

        // Encoder reversal flags
        const val kFrontLeftTurningEncoderReversed = true
        const val kBackLeftTurningEncoderReversed = true
        const val kFrontRightTurningEncoderReversed = true
        const val kBackRightTurningEncoderReversed = true

        const val kFrontLeftDriveEncoderReversed = true
        const val kBackLeftDriveEncoderReversed = true
        const val kFrontRightDriveEncoderReversed = false
        const val kBackRightDriveEncoderReversed = false

        // Absolute encoder ports
        const val kFrontLeftDriveAbsoluteEncoderPort = 0
        const val kBackLeftDriveAbsoluteEncoderPort = 2
        const val kFrontRightDriveAbsoluteEncoderPort = 1
        const val kBackRightDriveAbsoluteEncoderPort = 3

        // Absolute encoder reversal flags
        const val kFrontLeftDriveAbsoluteEncoderReversed = false
        const val kBackLeftDriveAbsoluteEncoderReversed = false
        const val kFrontRightDriveAbsoluteEncoderReversed = false
        const val kBackRightDriveAbsoluteEncoderReversed = false

        // Absolute encoder offsets (in radians)
        const val kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254
        const val kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252
        const val kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816
        const val kBackRightDriveAbsoluteEncoderOffsetRad = -4.811

        // Physical limits
        const val kPhysicalMaxSpeedMetersPerSecond = 5.0
        const val kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI

        // Teleoperated drive limits
        const val kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
        const val kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
        const val kTeleDriveMaxAccelerationUnitsPerSecond = 3.0
        const val kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0
    }

    object AutoConstants {
        const val kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
        const val kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
        const val kMaxAccelerationMetersPerSecondSquared = 3.0
        const val kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4

        const val kPXController = 1.5
        const val kPYController = 1.5
        const val kPThetaController = 3.0

        val kThetaControllerConstraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared
        )
    }

    object OIConstants {
        const val kDriverControllerPort = 0

        const val kDriverYAxis = 1
        const val kDriverXAxis = 0
        const val kDriverRotAxis = 4
        const val kDriverFieldOrientedButtonIdx = 1

        const val kDeadband = 0.05
    }
    }

