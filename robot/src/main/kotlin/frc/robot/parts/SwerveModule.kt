package frc.robot.parts

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.AbsoluteEncoder
import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.parts.MAXSwerveConfig


class SwerveModule (
    drivingCanId: Int,
    turningCanId: Int,
    turningCanEncoderId: Int,
    private var chassisAngularOffset: Double
) {
    private val drivingMotor: SparkMax = SparkMax(drivingCanId, SparkLowLevel.MotorType.kBrushless)
    private val turningMotor: SparkMax = SparkMax(turningCanId, SparkLowLevel.MotorType.kBrushless)

    private val drivingEncoder: RelativeEncoder = drivingMotor.encoder
    private val turningEncoder: RelativeEncoder = turningMotor.encoder
    private val turningCanEncoder: CANcoder = CANcoder(turningCanEncoderId, "rio")

    private val drivingClosedLoopController: SparkClosedLoopController = drivingMotor.closedLoopController
    private val turningClosedLoopController: SparkClosedLoopController = turningMotor.closedLoopController

    private var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())


    init {
        val config = MAXSwerveConfig()
        drivingMotor.configure(config.drivingConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
        turningMotor.configure(config.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
        desiredState.angle = Rotation2d(turningCanEncoder.position.valueAsDouble)
        drivingEncoder.position = 0.0
    }


    /** Used during simulation to get state. */
    fun getState(): SwerveModuleState {
        return SwerveModuleState(
            drivingEncoder.position,
            Rotation2d(turningEncoder.position - chassisAngularOffset)
        )
    }


    /** Return current position of the module. */
    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(
            drivingEncoder.position,
            Rotation2d(turningEncoder.position - chassisAngularOffset)
        )
    }


    /** Sets the desired state for the module with speed and angle */
    fun setDesiredState(desiredState: SwerveModuleState) {
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(turningEncoder.position))

        // Command driving and turning SPARKS towards their respective set-points.
        drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity)
        turningClosedLoopController.setReference(correctedDesiredState.angle.radians, ControlType.kPosition)

        this.desiredState = desiredState
    }


    /** Zeros all the SwerveModule encoders. */
    fun resetEncoders(){
        drivingEncoder.position = 0.0
    }


    /** Updates Turning encoder angle to CanEncoder position to reduce discrepency */
    fun updateTurningEncoder(){
        // pass
    }
}