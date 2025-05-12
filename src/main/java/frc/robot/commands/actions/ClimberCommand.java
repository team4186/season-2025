package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimberCommand extends Command {
  /* Intended Usage:
   * deploy w/ minimum voltage
   * on button press force motor back till limit switch enable    * */
  private final Climber climber;

  private int button_count = 0;
  private boolean isfinished = false;


  public ClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(this.climber);
  }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {}


  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    if (button_count == 1) {
      climber.deploy();
    } else if (button_count == 2) {
      isfinished = climber.pull();
    }
  }


  @Override
  public boolean isFinished() { return isfinished; }


  @Override
  public void end(boolean interrupted) {
    button_count = 0;
    climber.stop();
    isfinished = false;
  }


  public void button_detect(){
    button_count++;
  }
}
