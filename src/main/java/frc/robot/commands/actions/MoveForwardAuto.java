package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;


public class MoveForwardAuto extends Command {
    /**
     * Test this later in the pit. 
     * To earn a ranking point, all enabled robots must pass its starting line and score a coral on the reef.
     * Robot starting line is 7 ft, 4 in. Or 224 cm from the reef. 
     */

    private final SwerveSubsystem driveTrain;
    private boolean isFinished = false;
    private final double driveDistance;

    @Override
    public MoveForwardAuto(SwerveSubsystem driveTrain, double driveDistance) {
        this.driveTrain = driveTrain;
        this.distance = distance;
        this.currentDist = driveTrain.
        addRequirements(SwerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // have current dist, have desired dist, output a velocity output
        driveTrain.drive(this.driveVel, )
    }

    @Override
    public boolean isFinished() {
        // placeholder
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}