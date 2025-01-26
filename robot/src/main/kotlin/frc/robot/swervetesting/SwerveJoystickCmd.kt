package frc.robot.swervetesting

import java.util.function.Supplier
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.*
// Change below when we move these files to their actual directories.
import frc.robot.swervetesting.Constants.DriveConstants
import frc.robot.swervetesting.Constants.OIConstants
import frc.robot.swervetesting.SwerveSubsystem

class SwerveJoystickCmd: Command {
    private val swerveSubsystem: SwerveSubsystem
    private val xSpdFunction: () -> Double
    private val ySpdFunction: () -> Double
    private val turningSpdFunction: () -> Double
    private val fieldOrientedFunction: () -> Boolean
    private val xLimiter: SlewRateLimiter
    private val yLimiter: SlewRateLimiter
    private val turningLimiter: SlewRateLimiter

    constructor(swerveSubsystem: SwerveSubsystem, xSpdFunction: () -> Double, ySpdFunction: () -> Double, turningSpdFunction: () -> Double, fieldOrientedFunction: () -> Boolean) {
        this.swerveSubsystem = swerveSubsystem
        this.xSpdFunction = xSpdFunction
        this.ySpdFunction = ySpdFunction
        this.turningSpdFunction = turningSpdFunction
        this.fieldOrientedFunction = fieldOrientedFunction
        this.xLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        this.yLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        this.turningLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
    }

    override fun execute() {
        // 1. get real time joystick inputs.
        var xSpeed: Double = xSpdFunction()
        var ySpeed: Double = ySpdFunction()
        var turningSpeed: Double = turningSpdFunction()

        // 2. apply deadband.
        xSpeed =  if (abs(xSpeed) > OIConstants.kDeadband) xSpeed else 0.0
        ySpeed = if (abs(ySpeed) > OIConstants.kDeadband) ySpeed else 0.0
        turningSpeed = if(abs(turningSpeed) > OIConstants.kDeadband) turningSpeed else 0.0

        // 3. Make drive smoother.
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

        // 4. Construct desired chassis speeds.
        val chassisSpeeds: ChassisSpeeds
        if(fieldOrientedFunction()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d())
        } else {
            // Relative to the robot
            chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, turningSpeed)
        }

        // 5. Convert chassis speeds to individual module states.
        val moduleStates: Array<SwerveModuleState> = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)

        swerveSubsystem.setModuleStates(moduleStates)
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stopModules()
    }

    override fun isFinished(): Boolean {
        return false
    }
}
