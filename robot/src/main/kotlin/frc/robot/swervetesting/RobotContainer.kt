package frc.robot.swervetesting
//rishab y
import java.util.List
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.swervetesting.Constants.AutoConstants
import frc.robot.swervetesting.Constants.DriveConstants
import frc.robot.swervetesting.Constants.OIConstants
import frc.robot.swervetesting.SwerveJoystickCmd
import frc.robot.swervetesting.SwerveSubsystem


class mathRobotContainer {
    private val swerveSubsystem: SwerveSubsystem = SwerveSubsystem()

    //driverJoystick spelled incorrectly, use caution
    //configures joystick from port
    private val driverJoytick: Joystick = Joystick(OIConstants.kDriverControllerPort)

    //binds joystick axes to variables
    init {
        swerveSubsystem.setDefaultCommand(
            SwerveJoystickCmd(
                swerveSubsystem,
                { -driverJoytick.getRawAxis(OIConstants.kDriverYAxis) },
                { driverJoytick.getRawAxis(OIConstants.kDriverXAxis) },
                { driverJoytick.getRawAxis(OIConstants.kDriverRotAxis) },
                { !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) })
        )

        configureButtonBindings()
    }

    //button 2 zeroes the robot heading (assumed)
    private fun configureButtonBindings() {
        val buttonTrigger = Trigger {driverJoytick.getRawButton(2)}
        buttonTrigger.onTrue(InstantCommand({swerveSubsystem.zeroHeading()},swerveSubsystem))
    }
// auton calculations
    val autonomousCommand: Command
        get() {
            // 1. Create trajectory settings
            val trajectoryConfig: TrajectoryConfig = TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
                .setKinematics(DriveConstants.kDriveKinematics)

            // 2. Generate trajectory
            val trajectory: Trajectory = TrajectoryGenerator.generateTrajectory(
                Pose2d(0.0, 0.0, Rotation2d()),
                List.of(
                    Translation2d(1.0, 0.0),
                    Translation2d(1.0, -1.0)
                ),
                Pose2d(2.0, -1.0, Rotation2d.fromDegrees(180.0)),
                trajectoryConfig
            )

            // 3. Define PID controllers for tracking trajectory
            val xController: PIDController = PIDController(AutoConstants.kPXController, 0.0, 0.0)
            val yController: PIDController = PIDController(AutoConstants.kPYController, 0.0, 0.0)
            val thetaController: ProfiledPIDController = ProfiledPIDController(
                AutoConstants.kPThetaController, 0.0, 0.0, AutoConstants.kThetaControllerConstraints
            )
            thetaController.enableContinuousInput(-Math.PI, Math.PI)

            // 4. Construct command to follow trajectory
            val swerveControllerCommand: SwerveControllerCommand = SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem
            )

            // 5. Add some init and wrap-up, and return everything
            return SequentialCommandGroup(
                InstantCommand({swerveSubsystem.resetOdometry(trajectory.getInitialPose()) }),
                swerveControllerCommand,
                InstantCommand({swerveSubsystem.stopModules()}))
        }
}