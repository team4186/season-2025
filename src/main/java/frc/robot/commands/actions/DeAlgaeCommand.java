package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeAlgae;

public class DeAlgaeCommand extends Command {

    //TODO: deAlgae commands config buttons later

    /* Intended Usage:
    * Run with command while being held, alternate directions for 2 seconds in alternating directions.
    * intended to be used with whileTrue()
    *
    * isFinished -> when exit_timer reaches 500/ ~10 seconds
    * Interrupted -> Send reset command, stop motor, reset arm ...
    * */

    private final DeAlgae deAlgae;
    //private int timer = 0;
    private int exit_timer = 0;
    private int button_count = 0;
    private boolean isfinished = false;

    public DeAlgaeCommand(DeAlgae deAlgae) {
        this.deAlgae = deAlgae;
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
        // moves arm up and down while rolling possible algae until interrupted

        if (button_count == 2 ) {
            deAlgae.invertWheel();
        }
        else if (button_count == 3) {
            deAlgae.invertWheel();
        }
        else if(deAlgae.reset()){
            isfinished = true;
        }

        deAlgae.deploy();
        exit_timer++;


// TODO: floppy boy
//        else if (button_count == 1){
//            if(timer < 20){
//                deAlgae.runMotor_Up();
//            }
//            else if(timer < 50){
//                deAlgae.runMotor_Down();
//            }
//            else if(deAlgae.deploy()){
//               timer = 0;
//            }
//            timer++;
//            exit_timer++;
//        }
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
    public boolean isFinished() {return exit_timer >= 500 || isfinished;}


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
        deAlgae.stop();
    }

    public void button_detect(){
        button_count++;
    }
}
