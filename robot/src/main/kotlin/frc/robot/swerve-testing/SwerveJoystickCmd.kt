package frc.robot.swerve-testing

import java.util.function.Supplier
import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.*
// Change below when we move these files to their actual directories.
import frc.robot.swerve-test.Constants.DriveConstants
import frc.robot.swerve-test.Constants.OIConstants
import frc.robot.swerve-test..SwerveSubsystem

class SwerveJoystickCmd: CommandBase() {
    private val swerveSubsystem: SwerveSubsystem
    private val xSpdFunction: Supplier<Double>, ySpdFunction: Supplier<Double>, turningSpdFunction: Supplier<Double>
    private val fieldOrientedFunction: Supplier<Boolean>
    private val xLimiter: SlewRateLimiter, yLimiter: SlewRateLimiter, turningLimiter: SlewRateLimiter

    constructor(swerveSubsystem: SwerveSubsystem, xSpdFunction: Supplier<Double>, ySpdFunction: Supplier<Double>, turningSpdFunction: Supplier<Double>, fieldOrientedFunction: Supplier<Boolean>) {
        this.swerveSubsystem = swerveSubsystem
        this.xSpdFunction = xSpdFunction
        this.ySpdFunction = ySpdFunction
        this.turningSpdFunction = turningSpdFunction
        this.fieldOrientedFunction = fieldOrientedFunction
        this.xLimiter = xLimiter
        this.yLimiter = yLimiter
        this.turningLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)
        addRequirements(SwerveSubsystem)
    }

    override fun initialize() {
    }

    override fun execute() {
        // 1. get real time joystick inputs.
        xSpeed: double = xSpdFunction.get()
        ySpeed: double = ySpdFunction.get()
        turningSpeed: double = turningSpdFunction.get()

        // 2. apply deadband.
        xSpeed = abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0
        ySpeed = abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0
        turningSpeed = abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0

        // 3. Make drive smoother.
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

        // 4. Construct desired chassis speeds.

        ChassisSpeeds chassisSpeeds
        if(fieldOrientedFunction.get()) {
            chassisSpeeds = chassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d())
        } else {
            // Relative to the robot
            chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, turningSpeed)
        }

        // 5. Convert chassis speeds to individual module states.
        private val moduleStates: Array<SwerveModuleState> = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)

        swerveSubsystem.setModuleStates(moduleStates)
    }

    override fun end(interrupted: boolean) {
        swerveSubsystem.stopModules()
    }

    override fun isFinished(): boolean {
        return false
    }
}
