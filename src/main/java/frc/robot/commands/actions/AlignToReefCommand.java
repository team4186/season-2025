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

public class AlignToReefCommand extends Command {
  private boolean isFinished;
  private final boolean side;
  private final LimeLightRunner visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController turnPID;
  private final PIDController strafePID;
  private final PIDController distancePID;
  private double tagID = -1;
  private int currentIteration;


  public AlignToReefCommand(boolean side, LimeLightRunner visionSubsystem, SwerveSubsystem swerveSubsystem, PIDController turnPID, PIDController strafePID, PIDController distancePID) {
    this.isFinished = false;
    this.side = side;
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.turnPID = turnPID;
    this.strafePID = strafePID;
    this.distancePID = distancePID;
    this.currentIteration = 0;
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
    if (visionSubsystem.getTV(Constants.VisionConstants.LIME_LIGHT_NAME) && (LimelightHelpers.getFiducialID(Constants.VisionConstants.LIME_LIGHT_NAME) == tagID)) {
      isFinished = false;
      currentIteration = 0;
      Translation2d driveVec = new Translation2d(
          -distancePID.calculate(visionSubsystem.getHelperZOffset(), Constants.VisionConstants.BUFFER_DIST),
          strafePID.calculate(visionSubsystem.getHelperXOffset(), side ? Constants.VisionConstants.RIGHT_SCORE_OFFSET : Constants.VisionConstants.LEFT_SCORE_OFFSET));

      swerveSubsystem.drive(
          driveVec,
          turnPID.calculate(visionSubsystem.getAngleOffset(),
              0.0),
          false);
    } else {
      currentIteration++;
      if (currentIteration > 10) {
        isFinished=true;
      }
    }

  }


  @Override
  public boolean isFinished() {
    return isFinished;
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(), 0.0, false);
  }
}
