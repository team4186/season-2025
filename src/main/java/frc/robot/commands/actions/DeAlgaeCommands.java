package frc.robot.commands.actions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DeAlgae;
import frc.robot.subsystems.SwerveSubsystem;

public class DeAlgaeCommands extends Command {

    //TODO: deAlgae commands config buttons later

    private final DeAlgae deAlgae;
    private int timer = 0;
    private boolean isFinished = false;

    public DeAlgaeCommands(DeAlgae deAlgae) {
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
        /*TODO: first press sticks out arm
            second press makes arm go up and down while spinning motor
        */

        if(deAlgae.deploy() && !isFinished){
            if(timer < 20){
                deAlgae.runMotor_Up();
            }

            else if(timer < 50){
                deAlgae.runMotor_Down();
            }
            else {
                if(deAlgae.reset()){
                    isFinished = true;
                }
            }
            timer++;
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
    public boolean isFinished()
    {
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
    public void end(boolean interrupted)
    {
        deAlgae.reset();
    }
}
