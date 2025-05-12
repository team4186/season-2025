package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeProcessor;


public class AlgaeProcessorCommand extends Command {
  //TODO: deAlgae commands config buttons later

  /** Intended Usage:
   * Run with command while being held, alternate directions for 2 seconds in alternating directions.
   * intended to be used with whileTrue()
   * isFinished -> when exit_timer reaches 500/ ~10 seconds
   * Interrupted -> Send reset command, stop motor, reset arm ...
   */
  private final AlgaeProcessor algaeProcessor;
  private int button_count = 0;
  private boolean isFinished = false;
  private int ejectTimer = 0;
  private boolean ready = false;
  private boolean deployed = false;


  public AlgaeProcessorCommand( AlgaeProcessor algaeProcessor ) {
    this.algaeProcessor = algaeProcessor;
    addRequirements(this.algaeProcessor);
  }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {
  }


  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  // first press arm moves down and motor intakes, timer determines when to bring arm back, second press ejects.
  @Override
  public void execute() {
    // exit case
//        if (ejectTimer >= 60) {
//            algaeProcessor.wheelStop();
//            isFinished = algaeProcessor.reset();
//        }


    if ( button_count == 1 ) {
      algaeProcessor.cmd_runMotor_Down();
      algaeProcessor.intake();
    } else if ( button_count == 2 ) {
      algaeProcessor.stop();
      algaeProcessor.eject();
    } else if ( button_count == 3 ) {
      algaeProcessor.wheelStop();
    } else if ( button_count >= 4) {
      if ( button_count % 2 == 0) {
        isFinished = algaeProcessor.reset();
        algaeProcessor.wheelStop();
      } else {
        algaeProcessor.cmd_runMotor_Down();
        algaeProcessor.intake();
      }
    }
  }


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
    return isFinished;
  }


  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end( boolean interrupted ) {
    button_count = 0;
    isFinished = false;
    ejectTimer = 0;
    ready = false;
    deployed = false;
    algaeProcessor.stop();
  }


  public void button_detect() {
    button_count++;
  }
}
