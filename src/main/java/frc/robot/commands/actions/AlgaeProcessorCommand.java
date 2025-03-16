package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeProcessor;

public class AlgaeProcessorCommand extends Command {

    //TODO: deAlgae commands config buttons later

    /* Intended Usage:
     * Run with command while being held, alternate directions for 2 seconds in alternating directions.
     * intended to be used with whileTrue()
     *
     * isFinished -> when exit_timer reaches 500/ ~10 seconds
     * Interrupted -> Send reset command, stop motor, reset arm ...
     * */

    private final AlgaeProcessor algaeProcessor;
    private int exit_timer = 0;
    private int button_count = 0;
    private boolean isfinished = false;
    private int ejectTimer = 0;

    public AlgaeProcessorCommand(AlgaeProcessor algaeProcessor) {
        this.algaeProcessor = algaeProcessor;
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

    // first press arm moves down and motor intakes, second press brings arm back up, third press ejects.
    @Override
    public void execute() {
        if(ejectTimer >= 30){
            algaeProcessor.wheelStop();
            isfinished = algaeProcessor.reset();
        }

        if(button_count == 1){

            algaeProcessor.runMotor_Down();
            algaeProcessor.intake();
            exit_timer++;

            algaeProcessor.runMotor_Up();
            algaeProcessor.wheelStop();
            }
            else{
            algaeProcessor.eject();
            ejectTimer++;
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
    public boolean isFinished() {return isfinished;}


    /**
     * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
     * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
     * up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted)
    {
        button_count = 0;
        exit_timer = 0;
        algaeProcessor.stop();
        isfinished = false;
        ejectTimer = 0;
    }

    public void button_detect(){
        button_count++;
    }

}
