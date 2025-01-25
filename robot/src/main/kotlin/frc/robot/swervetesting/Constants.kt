package frc.robot.swervetesting
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import kotlin.math.PI


class Constants {
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
            val kTurningMotorGearRatio: Double = 1 / 18.0
            val kDriveEncoderRot2Meter: Double = kDriveMotorGearRatio * PI * kWheelDiameterMeters
            val kTurningEncoderRot2Rad: Double = kTurningMotorGearRatio * PI * kWheelDiameterMeters
            val kDriveEncoderRPM2MeterPerSec: Double = kDriveEncoderRot2Meter / 60
            val kTurningRPM2RadPerSec: Double = kTurningEncoderRot2Rad / 60
            val kPTurning: Double = 0.5
        }
    }

    object DriveConstants {
        val kTrackWidth: Double = Units.inchesToMeters(21.0)

        // Distance between right and left wheels
        val kWheelBase: Double = Units.inchesToMeters(25.5)

        // Distance between front and back wheels
        val kDriveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        )

        const val kFrontLeftDriveMotorPort: Int = 8
        const val kBackLeftDriveMotorPort: Int = 2
        const val kFrontRightDriveMotorPort: Int = 6
        const val kBackRightDriveMotorPort: Int = 4

        const val kFrontLeftTurningMotorPort: Int = 7
        const val kBackLeftTurningMotorPort: Int = 1
        const val kFrontRightTurningMotorPort: Int = 5
        const val kBackRightTurningMotorPort: Int = 3

        const val kFrontLeftTurningEncoderReversed: Boolean = true
        const val kBackLeftTurningEncoderReversed: Boolean = true
        const val kFrontRightTurningEncoderReversed: Boolean = true
        const val kBackRightTurningEncoderReversed: Boolean = true

        const val kFrontLeftDriveEncoderReversed: Boolean = true
        const val kBackLeftDriveEncoderReversed: Boolean = true
        const val kFrontRightDriveEncoderReversed: Boolean = false
        const val kBackRightDriveEncoderReversed: Boolean = false

        const val kFrontLeftDriveAbsoluteEncoderPort: Int = 0
        const val kBackLeftDriveAbsoluteEncoderPort: Int = 2
        const val kFrontRightDriveAbsoluteEncoderPort: Int = 1
        const val kBackRightDriveAbsoluteEncoderPort: Int = 3

        const val kFrontLeftDriveAbsoluteEncoderReversed: Boolean = false
        const val kBackLeftDriveAbsoluteEncoderReversed: Boolean = false
        const val kFrontRightDriveAbsoluteEncoderReversed: Boolean = false
        const val kBackRightDriveAbsoluteEncoderReversed: Boolean = false

        const val kFrontLeftDriveAbsoluteEncoderOffsetRad: Double = -0.254
        const val kBackLeftDriveAbsoluteEncoderOffsetRad: Double = -1.252
        const val kFrontRightDriveAbsoluteEncoderOffsetRad: Double = -1.816
        const val kBackRightDriveAbsoluteEncoderOffsetRad: Double = -4.811

        const val kPhysicalMaxSpeedMetersPerSecond: Double = 5.0
        const val kPhysicalMaxAngularSpeedRadiansPerSecond: Double = 2 * 2 * Math.PI

        const val kTeleDriveMaxSpeedMetersPerSecond: Double = kPhysicalMaxSpeedMetersPerSecond / 4
        const val kTeleDriveMaxAngularSpeedRadiansPerSecond: Double =  //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4
        const val kTeleDriveMaxAccelerationUnitsPerSecond: Double = 3.0
        const val kTeleDriveMaxAngularAccelerationUnitsPerSecond: Double = 3.0
    }


    object AutoConstants {
        const val kMaxSpeedMetersPerSecond: Double = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
        const val kMaxAngularSpeedRadiansPerSecond: Double =  //
            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
        const val kMaxAccelerationMetersPerSecondSquared: Double = 3.0
        const val kMaxAngularAccelerationRadiansPerSecondSquared: Double = Math.PI / 4
        const val kPXController: Double = 1.5
        const val kPYController: Double = 1.5
        const val kPThetaController: Double = 3.0

        val kThetaControllerConstraints: TrapezoidProfile.Constraints =  //
            TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared
            )
    }


    object OIConstants {
        const val kDriverControllerPort: Int = 0

        const val kDriverYAxis: Int = 1
        const val kDriverXAxis: Int = 0
        const val kDriverRotAxis: Int = 4
        const val kDriverFieldOrientedButtonIdx: Int = 1

        const val kDeadband: Double = 0.05
    }
}