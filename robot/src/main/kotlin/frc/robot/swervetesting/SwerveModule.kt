package frc.robot.swervetesting


import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogInput
import frc.robot.swervetesting.Constants.DriveConstants
import frc.robot.swervetesting.Constants.ModuleConstants

class SwerveModule (
    driveMotorId: Int,
    turningMotorId: Int,
    driveMotorReversed: Boolean,
    turningMotorReversed: Boolean,
    absoluteEncoderId: Int,
    private val absoluteEncoderOffsetRad: Double,
    private val absoluteEncoderReversed: Boolean
) {

    private val driveMotor: SparkMax = SparkMax(driveMotorId, SparkLowLevel.MotorType.kBrushless)
    private val turningMotor: SparkMax = SparkMax(turningMotorId, SparkLowLevel.MotorType.kBrushless)

    private val driveEncoder: RelativeEncoder = driveMotor.encoder
    private val turningEncoder: RelativeEncoder = turningMotor.encoder

    private val turningPIDController: PIDController = PIDController(
        ModuleConstants.kPTurning,
        0.0,
        0.0
    )

    private val absoluteEncoder: AnalogInput = AnalogInput(absoluteEncoderId)

    init{
        // Section removed from video because the functions do not exist anymore
        //driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter)
        //driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec)
        //turningEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad)
        //turningEncoder.setVelocityConversionFactor(Constants.kTurningRPM2RadPerSec)

        // Set motor inversion if needed
        //driveMotor.setInverted(driveMotorReversed)
        //turningMotor.setInverted(turningMotorReversed)

        // Optionally set up the PID controller settings
        //turningPIDController.setTolerance(ModuleConstants.kTurnToleranceDeg)

        turningPIDController.enableContinuousInput(-Math.PI, Math.PI)

        resetEncoders()
    }

    // Add methods to control the swerve module, e.g.:
    fun setDriveSpeed(speed: Double) {
        driveMotor.set(speed)
    }

    fun setTurningAngle(angle: Double) {
        // Convert the desired angle to PID control and apply to the turning motor
        val pidOutput = turningPIDController.calculate(turningEncoder.position, angle)
        turningMotor.set(pidOutput)
    }

    // Method to get the current turning angle from the absolute encoder
    fun getAbsoluteAngle(): Double {
        var angle = absoluteEncoder.voltage * 360.0 / 5.0 // assuming the encoder gives a value between 0-5V
        if (absoluteEncoderReversed) {
            angle = 360.0 - angle
        }
        return (angle + absoluteEncoderOffsetRad) % 360.0
    }

    fun resetEncoders() {
        driveEncoder.setPosition(0.0)
        turningEncoder.setPosition(getAbsoluteAngle())
    }

    fun getState(): SwerveModuleState {
        return SwerveModuleState(driveEncoder.velocity, Rotation2d(turningEncoder.position))
    }

    fun setDesiredState(state: SwerveModuleState) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        val state = SwerveModuleState.optimize(state, getState().angle)
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        turningMotor.set(turningPIDController.calculate(turningEncoder.position, state.angle.radians))
    }

    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(driveEncoder.position, Rotation2d(turningEncoder.position))
    }

    fun stop() {
        driveMotor.set(0.0)
        turningMotor.set(0.0)
    }
    // Optionally, you can add methods to get other encoder values, reset encoders, etc.
}



