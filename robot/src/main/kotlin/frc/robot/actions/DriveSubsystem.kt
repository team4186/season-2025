// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.actions

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.AnalogGyro
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.parts.SwerveModule


/** Represents a swerve drive style drivetrain.  */
class DriveSubsystem: SubsystemBase() {
    companion object {
        const val MAX_SPEED: Double = 3.0 // 3 meters per second
        const val MAX_ANGULAR_SPEED: Double = Math.PI // 1/2 rotation per second
    }

    // TODO: Walkthrough ids with team and set each id for each Swerve Module
    // ( driveMotorController, steerMotorController, encoderConstructor, constants, canbusName, driveTrainId )
    private val frontLeft = SwerveModule(1, 2, Constants.DriveConstants.)
    private val frontRight = SwerveModule(3, 4, Constants.DriveConstants.)
    private val backLeft = SwerveModule(5, 6, Constants.DriveConstants.)
    private val backRight = SwerveModule(7, 8, Constants.DriveConstants.)

    private val gyro = AnalogGyro(0)

    private val odometry = SwerveDriveOdometry(
        Constants.DriveConstants.kDriveKinematics,
        gyro.rotation2d,
        getSwervePositionArray()
    )


    init {
        gyro.reset()
    }


    override fun periodic() {
        odometry.update(
            gyro.rotation2d,
            getSwervePositionArray()
        )
    }


    private fun getSwervePositionArray(): Array<SwerveModulePosition> {
        return arrayOf(
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        )
    }


    fun getPose(): Pose2d {
        return odometry.poseMeters
    }


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
        val xSpeedDelivered: Double = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        val ySpeedDelivered: Double = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        val rotDelivered: Double = rot * DriveConstants.kMaxAngularSpeed

        val swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
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
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
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


    fun getTurnRate(): Double {
        val res = if (DriveConstants.kGyroReversed) -1.0 else 1.0
        return gyro.rate * res
    }
}
