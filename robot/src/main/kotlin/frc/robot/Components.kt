package frc.robot

import com.revrobotics.config.BaseConfig
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import frc.robot.parts.*

/**
 * The [Components] singleton can be used to configure and hold reference to hardware parts
 * used by the [Robot].
 *
 * The only gain here is organizational, as it avoids cluttering in the [Robot] class scope.
 */
object Components {
    object Propulsion {
        val LeftMotorSet = MotorSet(
            lead = SparkMax(8, SparkLowLevel.MotorType.kBrushless),
            follower0 = SparkMax(9, SparkLowLevel.MotorType.kBrushless),
            baseConfig = DefaultLeftConfig,
        )
        val RightMotorSet = MotorSet(
            lead = SparkMax(11, SparkLowLevel.MotorType.kBrushless),
            follower0 = SparkMax(10, SparkLowLevel.MotorType.kBrushless),
            baseConfig = DefaultRightConfig,
        )
    }
    object SimpleMotorTesting {
        val motor = SimpleMotor(12, SparkLowLevel.MotorType.kBrushless),
        val baseConfig = SimpleMotorConfig
    }
}
