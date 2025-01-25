package frc.robot.parts

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import frc.robot.actions.Drivetrain

class SwerveModule (
    driveMotorChannel: Int,
    turningMotorChannel: Int,
    driveEncoderChannelA: Int,
    driveEncoderChannelB: Int,
    turningEncoderChannelA: Int,
    turningEncoderChannelB: Int
) {

    private val kWheelRadius: Double = 0.0508
    private val kEncoderResolution: Int = 4096

    private val kModuleMaxAngularVelocity: Double = Drivetrain.MAX_ANGULAR_SPEED
    private val kModuleMaxAngularAcceleration: Double =
        2 * Math.PI // radians per second squared

    private val driveMotor: SparkMax = SparkMax(driveMotorChannel, SparkLowLevel.MotorType.kBrushless)
    private val turningMotor: SparkMax = SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless)

    private val driveEncoder: Encoder = Encoder(driveEncoderChannelA, driveEncoderChannelB)
    private val turningEncoder: Encoder = Encoder(turningEncoderChannelA, turningEncoderChannelB)

    // TODO: Adjust gains for our robot, these need to be updated
    private val drivePIDController: PIDController = PIDController(1.0, 0.0, 0.0)
    private val turningPIDController: ProfiledPIDController =
        ProfiledPIDController(
            1.0,
            0.0,
            0.0,
            TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)
        )

    // TODO: Adjust gains for our robot, these need to be updated
    private val driveFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(1.0, 3.0)
    private val turnFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(1.0, 0.5)

    init {

        // Set the distance per pulse for the drive encoder
        // Use distance traveled for one rotation of the wheel divided by the encoder resolution
        driveEncoder.distancePerPulse = 2 * Math.PI * kWheelRadius / kEncoderResolution

        // Set the angle in radians per pulse for the turning encoder
        // This is the angle of entire rotation divided by encoder resolution
        turningEncoder.distancePerPulse = 2 * Math.PI / kEncoderResolution

        // limit PID Controllers input range between -pi and pi and set to continuous
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI)
    }


    fun getState(): SwerveModuleState {
        return SwerveModuleState(
            driveEncoder.rate, Rotation2d(turningEncoder.distance)
        )
    }


    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(driveEncoder.distance, Rotation2d(turningEncoder.distance))
    }


    /** Sets the desired state for the module with speed and angle */
    fun setDesiredState(desiredState: SwerveModuleState) {
        val encoderRotation = Rotation2d(turningEncoder.distance)

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation)

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired direction
        // of travel that can occur when modules change directions. Results in smoother driving.
        desiredState.cosineScale(encoderRotation)

        // Calculate the drive output from the drive PID controller
        val driveOutput: Double = drivePIDController.calculate(driveEncoder.rate, desiredState.speedMetersPerSecond)
        val driveFeedforward: Double = driveFeedforward.calculate(desiredState.speedMetersPerSecond)

        // Calculate the turning motor output  from the turning PID controller
        val turnOutput: Double = turningPIDController.calculate(turningEncoder.distance, desiredState.angle.radians)
        val turnFeedforward: Double = turnFeedforward.calculate(turningPIDController.setpoint.velocity)

        driveMotor.setVoltage(driveOutput + driveFeedforward)
        turningMotor.setVoltage(turnOutput + turnFeedforward)
    }
}