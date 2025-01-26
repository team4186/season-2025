package frc.robot.swervetesting

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs

// Change below when we move these files to their actual directories.
import frc.robot.swervetesting.ModuleConstants.DriveConstants
import frc.robot.swervetesting.ModuleConstants.OIConstants
import frc.robot.swervetesting.SwerveSubsystem

class SwerveJoystickCmd(
    private val swerveSubsystem: SwerveSubsystem, // Primary constructor parameters
    private val xSpdFunction: () -> Double,
    private val ySpdFunction: () -> Double,
    private val turningSpdFunction: () -> Double,
    private val fieldOrientedFunction: () -> Boolean
) : Command() { // Initialize the superclass

    // Slew rate limiters
    private val xLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
    private val yLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
    private val turningLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)

    init {
        // Add subsystem requirements
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        // Initialization logic (if needed)
    }

    override fun execute() {
        // 1. Get real-time joystick inputs
        var xSpeed = xSpdFunction()
        var ySpeed = ySpdFunction()
        var turningSpeed = turningSpdFunction()

        // 2. Apply deadband
        xSpeed = if (abs(xSpeed) > OIConstants.kDeadband) xSpeed else 0.0
        ySpeed = if (abs(ySpeed) > OIConstants.kDeadband) ySpeed else 0.0
        turningSpeed = if (abs(turningSpeed) > OIConstants.kDeadband) turningSpeed else 0.0

        // 3. Make drive smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

        // 4. Construct desired chassis speeds
        val chassisSpeeds = if (fieldOrientedFunction()) {
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                turningSpeed,
                swerveSubsystem.getRotation2d()
            )
        } else {
            // Relative to the robot
            ChassisSpeeds(xSpeed, ySpeed, turningSpeed)
        }

        // 5. Convert chassis speeds to individual module states
        val moduleStates: Array<SwerveModuleState> = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)

        // 6. Set module states
        swerveSubsystem.setModuleStates(moduleStates)
    }

    override fun end(interrupted: Boolean) {
        // Stop the drivetrain when the command ends
        swerveSubsystem.stopModules()
    }

    override fun isFinished(): Boolean {
        return false // Command never finishes on its own
    }
}
