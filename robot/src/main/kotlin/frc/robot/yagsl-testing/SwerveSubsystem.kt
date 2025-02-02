package frc.robot.yagsl-testing

import java.io.File
import edu.wpi.first.wpilibj.Filesystem
import swervelib.parser.SwerveParser
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.yagsl-testing.Constants
import edu.wpi.first.units.Units.Meter

class SwerveSubsystem: SubsystemBase() {
    val swerveJsonDir: File = File(Filesystem.getDeployDirectory(), "swerve")
    val swerveDrive: SwerveDrive = SwerveDrive(swerveJsonDir).createSwerveDrive(maxSpeed)
    
    constructor() {
        try {
            // Currently going off of https://www.youtube.com/watch?v=2nXb7LXEtaY.
            swerveDrive = SwerveDrive(swerveJsonDir).createSwerveDrive(Constants.maxSpeed, Pose2d(Translate2d(), Rotation2d()))
        } catch (e: Exception) {
            println(e)
        }
    }
}
