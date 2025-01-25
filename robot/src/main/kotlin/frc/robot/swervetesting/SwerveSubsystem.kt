package frc.robot.swervetesting
// quick link: https://github.com/SeanSun6814/FRC0ToAutonomous
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
//below needs to be changed once we actually move our experimental code out of this swerve-test directory.
import frc.robot.swervetesting.Constants.DriveConstants

import frc.robot.swervetesting.SwerveModule

class SwerveSubsystem: SubsystemBase {
    
    private val SwerveModule frontLeft = SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    )

    private val SwerveModule frontRight = SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    )

    private val SwerveModule backLeft = SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    )

    private val SwerveModule backRight = SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    )

    private val desired

    private val AHRS gyro = AHRS(SPI.Port.xMXP)

    private val SwerveDriveOdometry odometer = SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d(0))

    constructor() {
        Thread {
        try {
            Thread.sleep(1000)
            zeroHeading()
        } catch (e: Exception) {
        }
        }.start()
    }

    fun zeroHeading() {
        gyro.reset()
    }

    fun getHeading(): double {
        return Math.IEEEremainder(gyro.getAngle(), 360)
    }

    fun getRotation2d(): Rotation2d {
        return Rotation2d.fromDegree(getHeading())
    }

    fun getPose(): Pose2d {
        return odometer.getPoseMeters()
    }

    fun resetOdometry(pose: Pose2d) {
        odometer.resetPosition(pose, getRotation2d())
    }

    override fun periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState())
        SmartDashboard.putNumber("Robot Heading", getHeading())
        SmartDashboard.putNumber("Robot Location", getPose().getTranslation().toString())
    }

    fun stopModules() {
        frontLeft.stop()
        frontRight.stop()
        backLeft.stop()
        backRight.stop()
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
    //Divides each element in desiredStates by MaxModuleSpeed to normalize.
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)

    //Takes the normalized Array and apply the desiredStates to each swerve module.
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        backLeft.setDesiredState(desiredStates[2])
        backRight.setDesiredState(desiredStates[3])
    }
}