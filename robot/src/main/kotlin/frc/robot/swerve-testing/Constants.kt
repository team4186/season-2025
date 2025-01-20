package frc.robot.`swerve-testing`
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
            val kTurningMotorGearRatio: Double =  1 / 18.0
            val kDriveEncoderRot2Meter: Double = kDriveMotorGearRatio * PI * kWheelDiameterMeters
            val kTurningEncoderRot2Rad: Double = kTurningMotorGearRatio * PI * kWheelDiameterMeters
            val kDriveEncoderRPM2MeterPerSec: Double =  kDriveEncoderRot2Meter / 60
            val kTurningRPM2RadPerSec: Double = kTurningEncoderRot2Rad / 60
            val kPTurning: Double = 0.5
        }
    }

}