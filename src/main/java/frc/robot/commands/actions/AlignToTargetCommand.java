package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.hardware.LimeLightRunner;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.hardware.LimeLightRunner;
import edu.wpi.first.math.controller.PIDController;


public class AlignToTargetCommand extends Command {
    private final LimeLightRunner visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private boolean isFinished;
    private double xOffset;
    private double distanceOffset;
    private PIDController turnPID;
    // Distance the drive train needs to stop from the wall.
    private double bufferDist;

    public AlignToTargetCommand(LimeLightRunner visionSubsystem, SwerveSubsystem swerveSubsystem, PIDController turnPID, double bufferDist) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.xOffset = visionSubsystem.getTagXOffset();
        this.distanceOffset = visionSubsystem.getDistance();
        this.turnPID = turnPID;
        this.bufferDist = bufferDist;
        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Add in if statement to check if both distance sensors are true. If not then it is unaligned.
            Translation2d driveVec = new Translation2d(xOffset, distanceOffset - bufferDist);
            swerveSubsystem.drive(driveVec, turnPID.calculate(xOffset, 0.0), true);

        // Gonna assume that the drive train locks by default with no input.
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
