package frc.robot.swervetesting
// quick link: https://github.com/SeanSun6814/FRC0ToAutonomous
//import com.kauailabs.navx.frc.AHRS
//below needs to be changed once we actually move our experimental code out of this swerve-test directory.

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogGyro
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.swervetesting.Constants.DriveConstants
import kotlin.math.IEEErem

class SwerveSubsystem() : SubsystemBase() {

    private val frontLeft = SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    )

    private val frontRight = SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    )

    private val backLeft = SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    )

    private val backRight = SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    )

    private val gyro = AnalogGyro(10)
    private val modulePositions: Array<SwerveModulePosition> = arrayOf(
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    )

    private val odometer = SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d(), modulePositions)

    init {
        Thread {
            try {
                Thread.sleep(1000)
                zeroHeading()
            } catch (e: Exception) {
                DriverStation.reportError("Error starting the swerve subsystem", e.stackTrace)
            }
        }.start()
    }

    // reset heading
    fun zeroHeading() {
        gyro.reset()
    }

    // Convert heading to angles.
    fun getHeading(): Double {
        return gyro.angle.IEEErem(360.0)
    }

    // Convert heading to rotation2d.
    fun getRotation2d(): Rotation2d {
        return Rotation2d.fromDegrees(getHeading())
    }

    //  return the x,y pos of robot on field in meters.
    fun getPose(): Pose2d {
        return odometer.getPoseMeters()
    }

    // It is what it says.
    fun resetOdometry(pose: Pose2d) {
        odometer.resetPosition(gyro.getRotation2d(), modulePositions, pose)
    }

    override fun periodic() {
        odometer.update(getRotation2d(), modulePositions)
        SmartDashboard.putNumber("Robot Heading", getHeading())
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString())
    }

    fun stopModules() {
        frontLeft.stop()
        frontRight.stop()
        backLeft.stop()
        backRight.stop()
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        //Divides each element in desiredStates by MaxModuleSpeed to normalize.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)

        //Takes the normalized Array and apply the desiredStates to each swerve module.
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        backLeft.setDesiredState(desiredStates[2])
        backRight.setDesiredState(desiredStates[3])
    }
}