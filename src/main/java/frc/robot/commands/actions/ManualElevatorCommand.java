package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command {
    private final Elevator elevatorSubsystem;
    private final int requestedLevel;
    // private final boolean is_finished = false;
    public ManualElevatorCommand(Elevator elevatorSubsystem, int requestedLevel) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.requestedLevel = requestedLevel;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (requestedLevel >= 0 && requestedLevel <= 5) {
           elevatorSubsystem.goToLevel(requestedLevel);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //elevatorSubsystem.reset();
        elevatorSubsystem.stopMotor();
    }
}
