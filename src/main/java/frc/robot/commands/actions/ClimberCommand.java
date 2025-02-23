package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public final class ClimberCommand extends Command {
    // TODO: NOT a new climber, you pass subsystem from robot container when calling command!
    private final Climber climber;
    private boolean isFinished = false;

    public ClimberCommand(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        // Init stuff here.
    }

    @Override
    public void execute() {
        // Loop stuff here.

        // deploy

        // wait for operator to put cage between bar

        // pull until robot is hanging

        // reset completed -> isFinished = True
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Idle mode is set to brake. However, this will not hold the climber in place.
        // However, brake does make the elevator descend slowly, so it wouldn't crash (this also resets encoders).
        // climber.stopMotor();
        climber.stop();
    }
}
