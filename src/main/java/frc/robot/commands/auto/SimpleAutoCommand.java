package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.actions.ElevatorCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;


public class SimpleAutoCommand extends Command {
  // WARNING: NOT a new elevator, you pass subsystem from robot container when calling command!

  private enum Task {
    Move,
    Align,
    Elevator,
    Eject,
    Reset,
    Stop
  }


  public SimpleAutoCommand(SwerveSubsystem driveSubSystem, Elevator elevatorSubsystemParam, int requestedLevel) {
    addRequirements();
  }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() { }


  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() { }


  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    //return taskState == ElevatorCommand.Task.STOP;
    return false;
  }


  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    // Idle mode is set to brake. However, this will not hold the elevator in place.
    // gravity will still continue to push the elevator down until the very bottom.
    // However, brake does make the elevator descend slowly, so it wouldn't crash (this also resets encoders).
    // elevatorSubsystem.stopMotor();

  }
}
