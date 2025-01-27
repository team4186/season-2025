package frc.robot

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

class Constants {
    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        val kMaxSpeedMetersPerSecond: Double = 4.8
        val kMaxAngularSpeed: Double = 2 * Math.PI // radians per second

        // Chassis configuration
        val kTrackWidth: Double = Units.inchesToMeters(26.5)

        // Distance between centers of right and left wheels on robot
        val kWheelBase: Double = Units.inchesToMeters(26.5)

        // Distance between front and back wheels on robot
        val kDriveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        )

        // Angular offsets of the modules relative to the chassis in radians
        val kFrontLeftChassisAngularOffset: Double = -Math.PI / 2
        val kFrontRightChassisAngularOffset: Double = 0.0
        val kBackLeftChassisAngularOffset: Double = Math.PI
        val kBackRightChassisAngularOffset: Double = Math.PI / 2

        // SPARK MAX CAN IDs
        val kFrontLeftDrivingCanId: Int = 11
        val kBackLeftDrivingCanId: Int = 13
        val kFrontRightDrivingCanId: Int = 15
        val kBackRightDrivingCanId: Int = 17

        val kFrontLeftTurningCanId: Int = 10
        val kBackLeftTurningCanId: Int = 12
        val kFrontRightTurningCanId: Int = 14
        val kBackRightTurningCanId: Int = 16

        val kGyroReversed: Boolean = false
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        val kDrivingMotorPinionTeeth: Int = 14

        // Calculations required for driving motor conversion factors and feed forward
        val kDrivingMotorFreeSpeedRps: Double = NeoMotorConstants.kFreeSpeedRpm / 60
        val kWheelDiameterMeters: Double = 0.0762
        val kWheelCircumferenceMeters: Double = kWheelDiameterMeters * Math.PI

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        val kDrivingMotorReduction: Double = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
        val kDriveWheelFreeSpeedRps: Double =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction
    }

    object OIConstants {
        val kDriverControllerPort: Int = 0
        val kDriveDeadband: Double = 0.05
    }

    object AutoConstants {
        val kMaxSpeedMetersPerSecond: Double = 3.0
        val kMaxAccelerationMetersPerSecondSquared: Double = 3.0
        val kMaxAngularSpeedRadiansPerSecond: Double = Math.PI
        val kMaxAngularSpeedRadiansPerSecondSquared: Double = Math.PI

        val kPXController: Double = 1.0
        val kPYController: Double = 1.0
        val kPThetaController: Double = 1.0

        // Constraint for the motion profiled robot angle controller
        val kThetaControllerConstraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        )
    }

    object NeoMotorConstants {
        val kFreeSpeedRpm: Double = 5676.0
    }
}
