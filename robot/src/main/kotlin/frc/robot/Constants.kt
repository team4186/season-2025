package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units

object Constants {
    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        val maxSpeedMetersPerSecond: Double = 4.8768 // meters per second
        val maxAngularSpeed: Double = 2 * Math.PI // radians per second

        // Chassis configuration
        val trackWidth: Double = Units.inchesToMeters(21.5)
        // Distance between centers of right and left wheels on robot
        val wheelBase: Double = Units.inchesToMeters(21.5)
        // Distance between front and back wheels on robot
        val driveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(wheelBase / 2, trackWidth / 2),
            Translation2d(wheelBase / 2, -trackWidth / 2),
            Translation2d(-wheelBase / 2, trackWidth / 2),
            Translation2d(-wheelBase / 2, -trackWidth / 2)
        )

        // Angular offsets of the modules relative to the chassis in radians
        val frontLeftChassisAngularOffset: Double = -Math.PI / 2
        val frontRightChassisAngularOffset: Double = 0.0
        val backLeftChassisAngularOffset: Double = Math.PI
        val backRightChassisAngularOffset: Double = Math.PI / 2

        // SPARK MAX CAN IDS
        val frontRightDrivingCanId: Int = 1
        val backRightDrivingCanId: Int = 3
        val backLeftDrivingCanId: Int = 5
        val frontLeftDrivingCanId: Int = 8

        val frontRightTurningCanId: Int = 2
        val backRightTurningCanId: Int = 4
        val backLeftTurningCanId: Int = 6
        val frontLeftTurningCanId: Int = 7

        val frontRightCanEncoderId: Int = 1
        val backRightCanEncoderId: Int = 2
        val backLeftCanEncoderId: Int = 3
        val frontLeftCanEncoderId: Int = 4

        val gyroChannelId: Int = 10
        val gyroReversed: Boolean = false
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        val drivingMotorPinionTeeth: Int = 14

        // Calculations required for driving motor conversion factors and feed forward
        val drivingMotorFreeSpeedRps: Double = NeoMotorConstants.freeSpeedRpm / 60
        val wheelDiameterMeters: Double = Units.inchesToMeters(4.0)
        val wheelCircumferenceMeters: Double = wheelDiameterMeters * Math.PI

        // 50 teeth on the wheel's bevel gear, 28 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        val drivingMotorReduction: Double = (50.0 * 28) / (drivingMotorPinionTeeth * 15)
        val driveWheelFreeSpeedRps: Double =
            (drivingMotorFreeSpeedRps * wheelCircumferenceMeters) / drivingMotorReduction
    }

    object OIConstants {
        val driverControllerPort: Int = 0
        val driveDeadband: Double = 0.1
    }

    /** Auto Parameters */
    object AutoConstants {
        val autoMaxSpeedMetersPerSecond: Double = 3.0
        val autoMaxAccelerationMetersPerSecondSquared: Double = 3.0
        val autoMaxAngularSpeedRadiansPerSecond: Double = Math.PI
        val autoMaxAngularSpeedRadiansPerSecondSquared: Double = Math.PI

        val autoPXController: Double = 1.0
        val autoPYController: Double = 1.0
        val autoPThetaController: Double = 1.0

        // Constraint for the motion profiled robot angle controller
        val autoThetaControllerConstraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
            autoMaxAngularSpeedRadiansPerSecond, autoMaxAngularSpeedRadiansPerSecondSquared
        )
    }

    object NeoMotorConstants {
        val freeSpeedRpm: Double = 6784.0
    }
}
