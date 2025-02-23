package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;



public class AlignToTargetCommand extends Command {
    private final Vision visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    // Probably configurable in limelight app thing with GUI.
    private final Translation2d limelightOffset;
    private boolean isFinished = false;

    public AlignToTargetCommand(Vision visionSubsystem, SwerveSubsystem swerveSubsystem, Translation2d limelightOffset) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.limelightOffset = limelightOffset;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
