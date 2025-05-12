package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class ElevatorReturnToDefault extends Command {
  private final Elevator elevatorSubsystem;


  public ElevatorReturnToDefault(Elevator elevatorSubsystemParam) {
    elevatorSubsystem = elevatorSubsystemParam;
    addRequirements(elevatorSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    elevatorSubsystem.goToLevel(0);
  }


  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtBottom();
  }


  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMotor();
  }
}
