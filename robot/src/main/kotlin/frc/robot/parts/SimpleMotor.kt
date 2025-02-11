package frc.robot.parts

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import frc.robot.Components.SingleMotorComponent


class SimpleMotor() {
    // private val component: SingleMotorTestingConfig = SingleMotorComponent.SingleMotor
    private val motor: SparkMax = SingleMotorComponent.SingleMotor.motor
    private val motorEncoder: RelativeEncoder = motor.encoder
    private val pid: PIDController = PIDController(0.2, 0.0, 0.0)
    private val MAX_SPEED: Double = 10.0

    init {
        // setup encoder
        zeroEncoder()
        pid.reset()
    }

    fun getState(): Double {
        return motorEncoder.position
    }

    fun zeroEncoder() {
        motorEncoder.position = 0.0
    }

    fun stopMotor() {
        motor.stopMotor()
        pid.reset()
    }

    fun move(forward: Double) {
        val speed = pid
                .calculate(forward)
                .coerceIn(-MAX_SPEED, MAX_SPEED)
        motor.set( speed )
        if ( motorEncoder.velocity <= 0.1 ) {
            zeroEncoder()
        }
    }
}
