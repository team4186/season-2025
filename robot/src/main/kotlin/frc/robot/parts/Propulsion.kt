package frc.robot.parts

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import frc.robot.Constants.ModuleConstants
import java.util.function.DoubleConsumer
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor


private val BaseConfig = SparkMaxConfig()
    .smartCurrentLimit(50)
    .idleMode(SparkBaseConfig.IdleMode.kCoast)

val DefaultLeftConfig: SparkBaseConfig = SparkMaxConfig()
    .apply(BaseConfig)
    .inverted(true)

val DefaultRightConfig: SparkBaseConfig = SparkMaxConfig()
    .apply(BaseConfig)
    .inverted(false)


val SimpleMotorConfig: SparkBaseConfig = SparkMaxConfig()
    .apply(BaseConfig)
    .inverted(false)


class SingleMotor(
    val motor: SparkMax,
    val baseConfig: SparkBaseConfig
) {
    init {
        motor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    fun accept(value: Double) {
        motor.set(value)
    }

    fun stop() {
        motor.stopMotor()
    }

    fun setIdleMode(mode: SparkBaseConfig.IdleMode) {
        motor.configure(
            SparkMaxConfig()
                .apply(baseConfig)
                .idleMode(mode),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
    }
}


class MotorSet(
    val lead: SparkMax,
    val follower0: SparkMax,
    val baseConfig: SparkBaseConfig,
) : DoubleConsumer {
    init {
        lead.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        SparkMaxConfig()
            .apply(baseConfig)
            .follow(lead)
            .let { follower0.configure(it, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) }
    }

    override fun accept(value: Double) {
        lead.set(value)
    }

    fun stop() {
        lead.stopMotor()
    }

    fun setIdleMode(mode: SparkBaseConfig.IdleMode) {
        lead.configure(
            SparkMaxConfig()
                .apply(baseConfig)
                .idleMode(mode),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
    }
}


// TODO: abstraction using Components
class MAXSwerveConfig {
    val drivingConfig: SparkMaxConfig = SparkMaxConfig()
    val turningConfig: SparkMaxConfig = SparkMaxConfig()

    private val drivingFactor: Double = (ModuleConstants.wheelDiameterMeters * Math.PI
            / ModuleConstants.drivingMotorReduction)
    private val turningFactor: Double = 2 * Math.PI
    private val drivingVelocityFeedForward: Double = 1.0 / ModuleConstants.driveWheelFreeSpeedRps

    init {
        drivingConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(50)
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0) // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: Adjust PID gains
            .pid(0.04, 0.0, 0.0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1.0, 1.0)

        turningConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(20)
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0) // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: Adjust PID gains
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, turningFactor)
    }
}
