package frc.robot.parts

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import frc.robot.Constants.ModuleConstants
import java.util.function.DoubleConsumer
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode


private val BaseConfig = SparkMaxConfig()
    .smartCurrentLimit(50)
    .idleMode(SparkBaseConfig.IdleMode.kCoast)

val DefaultLeftConfig: SparkBaseConfig = SparkMaxConfig()
    .apply(BaseConfig)
    .inverted(true)

val DefaultRightConfig: SparkBaseConfig = SparkMaxConfig()
    .apply(BaseConfig)
    .inverted(false)


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

    var drivingFactor: Double = (ModuleConstants.kWheelDiameterMeters * Math.PI
            / ModuleConstants.kDrivingMotorReduction)
    var turningFactor: Double = 2 * Math.PI
    var drivingVelocityFeedForward: Double = 1.0 / ModuleConstants.kDriveWheelFreeSpeedRps

    init {
        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0.0, 0.0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1.0, 1.0);

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, turningFactor);
    }
}
