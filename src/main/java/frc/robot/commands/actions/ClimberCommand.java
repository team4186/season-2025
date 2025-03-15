package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {

    /* Intended Usage:
     * deploy w/ minimum voltage
     * on button press force motor back till limit switch enable    * */

    private final Climber climber;

    public ClimberCommand(Climber climber) {
        this.climber = climber;
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

        //TODO: logic

    }


    @Override
    public void end(boolean interrupted)
    {
        climber.stop();
    }

}
