package frc.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.yagsl_testing.RobotContainer


class Robot : TimedRobot() {
    //private val joystick0 = Joystick(0)

//    private val drive = DifferentialDrive(
//        Components.Propulsion.LeftMotorSet,
//        Components.Propulsion.RightMotorSet,
//    )

    var instance: Robot? = null
    private val m_autonomousCommand: Command? = null

    private val m_robotContainer: RobotContainer? = null

    private val disabledTimer: Timer? = null

    init {
        instance = this
    }

    fun getInstance(): Robot? {
        return instance
    }

    private val autonomousChooser = SendableChooser<Command>()

    override fun robotInit() {
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

        with(autonomousChooser) {
            setDefaultOption("Nothing", null)
            SmartDashboard.putData("Autonomous Mode", this)
        }

        val m_robotContainer = RobotContainer()

    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

    }

    override fun autonomousInit() {
        autonomousChooser.selected?.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopInit() {
    }

    override fun teleopPeriodic() {
//        manualDrive(
//            forward = joystick0.y,
//            turn = joystick0.twist,
//            drive = { forward, turn -> drive.arcadeDrive(forward, turn, true) }
//        )
    }

    override fun teleopExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
    }
}
