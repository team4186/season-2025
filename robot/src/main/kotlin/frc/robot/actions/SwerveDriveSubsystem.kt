// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.actions

import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import frc.robot.parts.SwerveModule


/** Represents a swerve drive style drivetrain.  */
class SwerveDriveSubsystem: SubsystemBase() {
    companion object {
        val MAX_SPEED: Double = DriveConstants.maxSpeedMetersPerSecond // Meters per Second
        val MAX_ANGULAR_SPEED: Double = DriveConstants.maxAngularSpeed // Degree rotation per second
    }

    // TODO: Walkthrough ids with team and set each id for each Swerve Module
    // ( driveMotorController, steerMotorController, encoderConstructor, constants, canbusName, driveTrainId )
    private val frontLeft = SwerveModule(
        DriveConstants.frontLeftDrivingCanId,
        DriveConstants.frontLeftTurningCanId,
        DriveConstants.frontLeftChassisAngularOffset
    )
    private val frontRight = SwerveModule(
        DriveConstants.frontRightDrivingCanId,
        DriveConstants.frontRightTurningCanId,
        DriveConstants.frontRightChassisAngularOffset
    )
    private val backLeft = SwerveModule(
        DriveConstants.backLeftDrivingCanId,
        DriveConstants.backLeftTurningCanId,
        DriveConstants.backLeftChassisAngularOffset
    )
    private val backRight = SwerveModule(
        DriveConstants.backRightDrivingCanId,
        DriveConstants.backRightTurningCanId,
        DriveConstants.backRightChassisAngularOffset
    )
    // private val gyro = AnalogGyro(DriveConstants.gyroChannelId)
    // TODO: Double check Canbus naming
    private val gyro: Pigeon2 = Pigeon2(DriveConstants.gyroChannelId, "rio")
    private val odometry = SwerveDriveOdometry(
        DriveConstants.driveKinematics,
        gyro.rotation2d,
        getSwervePositionArray()
    )


    init {
        gyro.reset()
    }


    /** Update odometry every periodic time-step. */
    override fun periodic() {
        odometry.update(
            gyro.rotation2d,
            getSwervePositionArray()
        )
    }


    /** Returns SwervePositionArray object */
    private fun getSwervePositionArray(): Array<SwerveModulePosition> {
        return arrayOf(
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        )
    }


    /** Returns current pose. */
    fun getPose(): Pose2d {
        return odometry.poseMeters
    }


    /** Reset Position using pose. */
    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            gyro.rotation2d,
            getSwervePositionArray(),
            pose
        )
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    fun drive(
        xSpeed: Double,
        ySpeed: Double,
        rot: Double,
        fieldRelative: Boolean,
    ) {
        val xSpeedDelivered: Double = xSpeed * DriveConstants.maxSpeedMetersPerSecond
        val ySpeedDelivered: Double = ySpeed * DriveConstants.maxSpeedMetersPerSecond
        val rotDelivered: Double = rot * DriveConstants.maxAngularSpeed

        val swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
            if (fieldRelative) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    gyro.rotation2d
                )
            } else {
                ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            }
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond
        )
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        backLeft.setDesiredState(swerveModuleStates[2])
        backRight.setDesiredState(swerveModuleStates[3])
    }


    /** Set wheels into an X formation to prevent movement. */
    fun setX(){
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        backLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        backRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }


    fun resetEncoders(){
        frontLeft.resetEncoders()
        frontRight.resetEncoders()
        backLeft.resetEncoders()
        backRight.resetEncoders()
    }


    fun zeroHeading() {
        gyro.reset()
    }


    fun getHeading(): Double {
        return gyro.rotation2d.degrees
    }


    // TODO: gyro.rate deprecates 2026, needs to be updated
    fun getTurnRate(): Double {
        val res: Double = if (DriveConstants.gyroReversed) -1.0 else 1.0
        return gyro.rate * res
    }
}
