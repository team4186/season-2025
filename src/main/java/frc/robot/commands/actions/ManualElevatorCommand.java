package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class ManualElevatorCommand extends Command {

  private final Elevator elevatorSubsystem;
  private final int requestedLevel;
  private boolean is_finished;


  public ManualElevatorCommand(Elevator elevatorSubsystem, int requestedLevel) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.requestedLevel = requestedLevel;
    is_finished = false;
  }


  @Override
  public void initialize() { }


  @Override
  public void execute() {
    if (requestedLevel >= 0 && requestedLevel <= 5) {
      elevatorSubsystem.goToLevel(requestedLevel);
    } else {
      is_finished = true;
    }
  }


  @Override
  public boolean isFinished() {
    return is_finished;
  }


  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMotor();
  }
}
