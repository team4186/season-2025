package frc.robot.yagsl_testing


//import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.swervedrive.SwerveSubsystem
import frc.robot.yagsl_testing.Constants.OperatorConstants
import swervelib.SwerveInputStream
import java.io.File
import kotlin.math.cos
import kotlin.math.sin
import frc.robot.Robot


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    //val driverXbox: CommandXboxController = CommandXboxController(0)
    val joystick: CommandJoystick = CommandJoystick(0)

    // The robot's subsystems and commands are defined here...
    private val drivebase = SwerveSubsystem(
        File(
            Filesystem.getDeployDirectory(),
            "swerve/neo"
        )
    )

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    var driveAngularVelocity: SwerveInputStream = SwerveInputStream.of(
        drivebase.swerveDrive,
        { joystick.y * -1 },
        { joystick.x * -1 })
        .withControllerRotationAxis { joystick.twist}
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true)

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */

    //useful for controllers, not applicable for 1 joysticks, needs second joystick
//    var driveDirectAngle: SwerveInputStream = driveAngularVelocity.copy().withControllerHeadingAxis(
//        { driverXbox.rightX },
//        { driverXbox.rightY })
//        .headingWhile(true)

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    var driveRobotOriented: SwerveInputStream = driveAngularVelocity.copy().robotRelative(true)
        .allianceRelativeControl(false)

//    var driveAngularVelocityKeyboard: SwerveInputStream = SwerveInputStream.of(
//        drivebase.swerveDrive,
//        { -joystick.y },
//        { -joystick.x })
//        .withControllerRotationAxis {
//            joystick.getRawAxis(
//                2
//            )
//        }
//        .deadband(OperatorConstants.DEADBAND)
//        .scaleTranslation(0.8)
//        .allianceRelativeControl(true)
//
//    // Derive the heading axis with math!
//    var driveDirectAngleKeyboard: SwerveInputStream = driveAngularVelocityKeyboard.copy()
//        .withControllerHeadingAxis(
//            {
//                sin(
//                    joystick.getRawAxis(
//                        2
//                    ) *
//                            Math.PI
//                ) *
//                        (Math.PI *
//                                2)
//            },
//            {
//                cos(
//                    joystick.getRawAxis(
//                        2
//                    ) *
//                            Math.PI
//                ) *
//                        (Math.PI *
//                                2)
//            })
//        .headingWhile(true)

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the trigger bindings
        configureBindings()
        DriverStation.silenceJoystickConnectionWarning(true)
        //NamedCommands.registerCommand("test", Commands.print("I EXIST"))
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary predicate, or via the
     * named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        //direct angle not functional
        //val driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle)

        val driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity)
        val driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented)

        // direct angle not functional
//        val driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
//            driveDirectAngle
//        )

//        val driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard)
//        val driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard)
//        val driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
//            driveDirectAngleKeyboard
//        )

        if (RobotBase.isSimulation()) {
            drivebase.defaultCommand = driveFieldOrientedDirectAngleKeyboard
        } else {
            drivebase.defaultCommand = driveFieldOrientedAnglularVelocity
        }

        // buttons could be set up better

        //simulation not important right now
//        if (Robot.isSimulation()) {
//            joystick.button(7).onTrue(Commands.runOnce({ drivebase.resetOdometry(Pose2d(3.0, 3.0, Rotation2d())) }))
//            joystick.button(8).whileTrue(drivebase.sysIdDriveMotorCommand())
//        }

        if (DriverStation.isTest()) {
            drivebase.defaultCommand = driveFieldOrientedAnglularVelocity // Overrides drive command above!

            joystick.button(3).whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            joystick.button(4).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2))
            joystick.button(5).onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            joystick.button(6).whileTrue(drivebase.centerModulesCommand())
            joystick.button(7).onTrue(Commands.none())
            joystick.button(8).onTrue(Commands.none())
        } else {
            //drivebase.defaultCommand = driveCommand(joystick.getX(), joystick.getY(), )
            joystick.button(9).onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
//            joystick.button(10).onTrue(Commands.runOnce({ drivebase.addFakeVisionReading() }))
            joystick.button(11).whileTrue(
                drivebase.driveToPose(
                    Pose2d(Translation2d(4.0, 4.0), Rotation2d.fromDegrees(0.0))
                )
            )
            joystick.button(5).whileTrue(Commands.none())
            joystick.button(6).whileTrue(Commands.none())
            joystick.button(7).whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            joystick.button(8).onTrue(Commands.none())
        }
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() =// An example command will be run in autonomous
            drivebase.getAutonomousCommand("New Auto")

    fun setMotorBrake(brake: Boolean) {
        drivebase.setMotorBrake(brake)
    }
}