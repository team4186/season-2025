package frc.robot.swerve-testing
//rishab
import java.util.List
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.Constants.AutoConstants
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.OIConstants
import frc.robot.commands.SwerveJoystickCmd
import frc.robot.subsystems.SwerveSubsystem

class RobotContainer {
    private val swerveSubsystem: SwerveSubsystem = SwerveSubsystem()

    //driverJoytick spelled incorrectly, use caution
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
        JoystickButton(driverJoytick, 2).whenPressed { swerveSubsystem.zeroHeading() }
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
                Pose2d(0, 0, Rotation2d(0)),
                List.of(
                    Translation2d(1, 0),
                    Translation2d(1, -1)
                ),
                Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig
            )

            // 3. Define PID controllers for tracking trajectory
            val xController: PIDController = PIDController(AutoConstants.kPXController, 0, 0)
            val yController: PIDController = PIDController(AutoConstants.kPYController, 0, 0)
            val thetaController: ProfiledPIDController = ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
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
                InstantCommand { swerveSubsystem.resetOdometry(trajectory.getInitialPose()) },
                swerveControllerCommand,
                InstantCommand { swerveSubsystem.stopModules() })
        }
}