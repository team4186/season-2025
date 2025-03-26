package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.hardware.LimeLightRunner;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;


public class AlignToTargetCommand extends Command {

    private final LimeLightRunner visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private boolean isFinished;
    private double xOffset;
    private double distanceOffset;
    private final PIDController turnPID;
    private final PIDController strafePID;
    private final PIDController distancePID;
    // Distance the drive train needs to stop from the wall.
    private final double bufferDist;


    public AlignToTargetCommand(LimeLightRunner visionSubsystem, SwerveSubsystem swerveSubsystem, PIDController turnPID, PIDController strafePID, PIDController distancePID, double bufferDist) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;;
        this.turnPID = turnPID;
        this.strafePID = strafePID;
        this.distancePID = distancePID;
        this.bufferDist = bufferDist;
        addRequirements(this.swerveSubsystem);
    }


    @Override
    public void initialize() { }


    @Override
    public void execute() {
        // Add in if statement to check if both distance sensors are true. If not then it is unaligned.
        if( visionSubsystem.hasTargetTag() ) {
            Translation2d driveVec = new Translation2d(strafePID.calculate(visionSubsystem.getXOffset(), 0.0), distancePID.calculate(visionSubsystem.getZOffset(), bufferDist));
            swerveSubsystem.drive(driveVec, turnPID.calculate(visionSubsystem.getThetaOffset(), 0.0), true);
        }
    }

    @Override
    public boolean isFinished() {
        return closeEnough( visionSubsystem.getXOffset() ) && closeEnough( visionSubsystem.getThetaOffset() ) && closeEnough( visionSubsystem.getZOffset() );
    }


    // The numbers are arbituary, it can be changed later.
    public boolean closeEnough(double input) {
        return input >= -0.2 && input <= 0.2;
    }


    @Override
    public void end(boolean interrupted) { }
}
