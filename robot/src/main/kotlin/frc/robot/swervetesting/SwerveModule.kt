package frc.robot.swervetesting

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogInput
import frc.robot.swervetesting.Constants.DriveConstants
import frc.robot.swervetesting.Constants.ModuleConstants

class SwerveModule(
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

    private val driveEncoder = driveMotor.encoder
    private val turningEncoder = turningMotor.encoder
    private val turningPIDController: PIDController = PIDController(
        ModuleConstants.kPTurning,
        0.0,
        0.0
    )

    private val absoluteEncoder: AnalogInput = AnalogInput(absoluteEncoderId)

    init {
        // Section removed from video because the functions do not exist anymore
        //TODO: update to usable code


        driveMotor.configure(
            SparkMaxConfig()
                .apply {
                    inverted(driveMotorReversed)
                    encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                    encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
                },
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        turningMotor.configure(
            SparkMaxConfig()
                .apply {
                    inverted(turningMotorReversed)
                    encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                    encoder.velocityConversionFactor(ModuleConstants.kTurningRPM2RadPerSec)
                },
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        //Optionally set up the PID controller settings
        //line 54 could possibly require the constant kPTurning rather than kTurnToleranceDeg
        turningPIDController.setTolerance(ModuleConstants.kPTurning)

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



