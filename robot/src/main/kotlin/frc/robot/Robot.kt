package frc.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.robot.actions.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler


class Robot : TimedRobot() {
    private val joystick0 = Joystick(0)
    private val swerve: Drivetrain = Drivetrain()

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private val xSpeedLimiter = SlewRateLimiter(3.0)
    private val ySpeedLimiter = SlewRateLimiter(3.0)
    private val rotLimiter = SlewRateLimiter(3.0)

    private val drive = DifferentialDrive(
        Components.Propulsion.LeftMotorSet,
        Components.Propulsion.RightMotorSet,
    )

    private val autonomousChooser = SendableChooser<Command>()

    override fun robotInit() {
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

        with(autonomousChooser) {
            setDefaultOption("Nothing", null)
            SmartDashboard.putData("Autonomous Mode", this)
        }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autonomousChooser.selected?.schedule()
    }

    override fun autonomousPeriodic() {
        driveWithJoystick(false)
        swerve.updateOdometry()
    }

    override fun autonomousExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopInit() {
    }

    override fun teleopPeriodic() {
        /*
        manualDrive(

            forward = joystick0.y,
            turn = joystick0.twist,
            drive = { forward, turn -> drive.arcadeDrive(forward, turn, true) }
        )*/

        driveWithJoystick(true)

    }

    override fun teleopExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
    }

    private fun driveWithJoystick(fieldRelative: Boolean) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        val xSpeed: Double =
            (-xSpeedLimiter.calculate(MathUtil.applyDeadband(joystick0.x, 0.02))
                    * Drivetrain.MAX_SPEED)

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        val ySpeed: Double =
            (-ySpeedLimiter.calculate(MathUtil.applyDeadband(joystick0.y, 0.02))
                    * Drivetrain.MAX_SPEED)

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        val rot: Double =
            (-rotLimiter.calculate(MathUtil.applyDeadband(joystick0.twist, 0.02))
                    * Drivetrain.MAX_ANGULAR_SPEED)

        swerve.drive(xSpeed, ySpeed, rot, fieldRelative, period)
    }
}
