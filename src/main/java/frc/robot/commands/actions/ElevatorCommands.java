package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;


public final class ElevatorCommands extends Command {
    // TODO: NOT a new elevator, you pass subsystem from robot container when calling command!
    private final Elevator elevatorSubsystem;
    private boolean isFinished = false;

    public ElevatorCommands(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        // Init stuff here.
    }

    @Override
    public void execute() {
        // Loop stuff here.
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Idle mode is set to brake. However, this will not hold the elevator in place.
        // gravity will still continue to push the elevator down until the very bottom.
        // However, brake does make the elevator descend slowly, so it wouldn't crash (this also resets encoders).
        elevatorSubsystem.stopMotor();
    }
}
