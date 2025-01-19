// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.actions

import frc.robot.parts.SwerveModule
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.AnalogGyro

/** Represents a swerve drive style drivetrain.  */
class Drivetrain {
    companion object {
        const val MAX_SPEED: Double = 3.0 // 3 meters per second
        const val MAX_ANGULAR_SPEED: Double = Math.PI // 1/2 rotation per second
    }

    private val frontLeftLocation = Translation2d(0.381, 0.381)
    private val frontRightLocation = Translation2d(0.381, -0.381)
    private val backLeftLocation = Translation2d(-0.381, 0.381)
    private val backRightLocation = Translation2d(-0.381, -0.381)

    // TODO: Walkthrough ids with team and set each id for each Swerve Module
    // ( driveMotorController, steerMotorController, encoderConstructor, constants, canbusName, driveTrainId )
    private val frontLeft = SwerveModule(1, 2, 0, 1, 2, 3)
    private val frontRight = SwerveModule(3, 4, 4, 5, 6, 7)
    private val backLeft = SwerveModule(5, 6, 8, 9, 10, 11)
    private val backRight = SwerveModule(7, 8, 12, 13, 14, 15)

    private val gyro = AnalogGyro(0)

    private val kinematics = SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    )

    private val odometry = SwerveDriveOdometry(
        kinematics,
        gyro.rotation2d,
        getSwervePositionArray()
    )


    init {
        gyro.reset()
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
        periodSeconds: Double
    ) {
        val swerveModuleStates =
            kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    if (fieldRelative)
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            xSpeed,
                            ySpeed,
                            rot,
                            gyro.rotation2d
                        )
                    else
                        ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds
                )
            )

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED)

        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        backLeft.setDesiredState(swerveModuleStates[2])
        backRight.setDesiredState(swerveModuleStates[3])
    }


    fun getSwervePositionArray(): Array<SwerveModulePosition> {
        return arrayOf(
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        )
    }

    /** Updates the field relative position of the robot.  */
    fun updateOdometry() {
        odometry.update(
            gyro.rotation2d,
            getSwervePositionArray()
        )
    }
}
