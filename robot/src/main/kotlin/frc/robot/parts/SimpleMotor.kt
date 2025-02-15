package frc.robot.parts

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Components.SingleMotorComponent


class SimpleMotor() {
    // private val component: SingleMotorTestingConfig = SingleMotorComponent.SingleMotor
    private val motor: SparkMax = SingleMotorComponent.SingleMotor.motor
    private val motorEncoder: RelativeEncoder = motor.encoder
    private val pid: PIDController = PIDController(0.2, 0.0, 0.0)
    private val MAX_SPEED: Double = 1.0

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

    fun move(speed: Double) {
        val adjSpeed = pid
                .calculate(speed)
                .coerceIn(-MAX_SPEED, MAX_SPEED)
        SmartDashboard.putNumber("controller_motor_speed_pidadj", adjSpeed )
        motor.set( adjSpeed )
        SmartDashboard.putNumber("controller_motor_velocity", motorEncoder.velocity )
        if ( motorEncoder.velocity <= 0.8 ) {
            zeroEncoder()
        }
    }
}
