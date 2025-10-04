package frc.robot.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.hardware.LimeLightRunner;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * IMPORTANT: Tune PIDs or this will crash out.
 */
public class AlignToTargetCommand extends Command {
  private boolean isFinished;
  private final LimeLightRunner visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController turnPID;
  private final PIDController strafePID;
  private final PIDController distancePID;

  // Just zero right now.
  // If you're wondering why this is a double, it is because getFiducialID returns a double.
  private double tagID = -1;


  public AlignToTargetCommand(LimeLightRunner visionSubsystem, SwerveSubsystem swerveSubsystem, PIDController turnPID, PIDController strafePID, PIDController distancePID) {
    this.isFinished = false;
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.turnPID = turnPID;
    this.strafePID = strafePID;
    this.distancePID = distancePID;
    addRequirements(swerveSubsystem);
  }


  @Override
  public void initialize() {
    // Play around with the tolerance a bit when testing.
    turnPID.setTolerance(Constants.VisionConstants.TURN_TOLERANCE);
    distancePID.setTolerance(Constants.VisionConstants.DISTANCE_TOLERANCE);
    strafePID.setTolerance(Constants.VisionConstants.STRAFE_TOLERANCE);
    tagID = LimelightHelpers.getFiducialID(Constants.VisionConstants.LIME_LIGHT_NAME);
  }


  @Override
  public void execute() {
    // Only move when a tag is visible and the tag in frame is the one that the limelight first saw when the command is triggered.
    if ( visionSubsystem.getTV(Constants.VisionConstants.LIME_LIGHT_NAME)
        && LimelightHelpers.getFiducialID(Constants.VisionConstants.LIME_LIGHT_NAME) == tagID) {
      Translation2d driveVec = new Translation2d(
          distancePID.calculate(visionSubsystem.getHelperZOffset(), Constants.VisionConstants.BUFFER_DIST),
          strafePID.calculate(visionSubsystem.getHelperXOffset(),0.0));

      swerveSubsystem.drive(
          driveVec,
          turnPID.calculate(visionSubsystem.getAngleOffset(),
              0.0),
          false);
    } else {
      /**
       * This also works considering that once the robot is aligned, it would be too close to the wall
       * to see the apriltag, which would trigger this condition.
       */
      swerveSubsystem.drive(new Translation2d(), 0.0, false);
      isFinished = true;
    }

  }


  @Override
  public boolean isFinished() {
    return isFinished;
  }


  @Override
  public void end(boolean interrupted) {
    // stops driving
    swerveSubsystem.drive(new Translation2d(), 0.0, false);
  }
}
