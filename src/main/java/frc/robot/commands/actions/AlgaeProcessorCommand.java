//package frc.robot.commands.actions;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
//import frc.robot.subsystems.AlgaeProcessor;
//
//public class AlgaeProcessorCommand extends Command {
//
//    //TODO: algaeProcessor commands config buttons later
//
//    /* Intended Usage:
//     * Run with command while being held, alternate directions for 2 seconds in alternating directions.
//     * intended to be used with whileTrue()
//     *
//     * isFinished -> when exit_timer reaches 500/ ~10 seconds
//     * Interrupted -> Send reset command, stop motor, reset arm ...
//     * */
//
//    private final AlgaeProcessor algaeProcessor;
//    //private int timer = 0;
//    private int exit_timer = 0;
//    private int button_count = 0;
//    private boolean isfinished = false;
//    private boolean deployed = false;
//
//    public AlgaeProcessorCommand(AlgaeProcessor algaeProcessor) {
//        this.algaeProcessor = algaeProcessor;
//    }
//
//
//    /**
//     * The initial subroutine of a command.  Called once when the command is initially scheduled.
//     */
//    @Override
//    public void initialize() {}
//
//
//    /**
//     * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//     * until {@link #isFinished()}) returns true.)
//     */
//    @Override
//    public void execute() {
//
//        // One stroke AlgaeProcessor
//        if(button_count >= 2){
//            deployed = true;
//        }
//
//        if(deployed || exit_timer >= 150){
//            isfinished = algaeProcessor.reset();
//        }
//
//        else if(algaeProcessor.deploy()){
//            deployed = true;
//        }
//
//        exit_timer ++;
//
//
//
//        //TODO: working two stage algaeProcessor
////        if (button_count == 1 ) {
////            algaeProcessor.runMotor_Up();
////        }
////
////        else if(button_count >= 2 || exit_timer >= 500) {
////            isfinished = algaeProcessor.reset();
////
////        }
////
////        exit_timer++;
//
////   TODO: these are not expected to function correctly, both were scrapped halfway through due to simplifications on how algaeProcessor worked
//
//// TODO: stages
////        if (button_count == 1 ) {
////            algaeProcessor.runMotor_Up(Constants.AlgaeProcessorConstants.DE_ALGAE_MIN_ANGLE);
////        }
////
////        else if(button_count == 2){
////            algaeProcessor.runMotor_Up(Constants.AlgaeProcessorConstants.DE_ALGAE_MAX_ANGLE);
////        }
////
////        else if(button_count >= 3 || exit_timer >= 500) {
////            isfinished = algaeProcessor.reset();
////
////        }
//
//// TODO: floppy boy
////        else if (button_count == 1){
////            if(timer < 20){
////                algaeProcessor.runMotor_Up();
////            }
////            else if(timer < 50){
////                algaeProcessor.runMotor_Down();
////            }
////            else if(algaeProcessor.deploy()){
////               timer = 0;
////            }
////            timer++;
////            exit_timer++;
////        }
//    }
//
//
//    /**
//     * <p>
//     * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
//     * the scheduler will call its {@link #end(boolean)} method.
//     * </p><p>
//     * Returning false will result in the command never ending automatically. It may still be cancelled manually or
//     * interrupted by another command. Hard coding this command to always return true will result in the command executing
//     * once and finishing immediately. It is recommended to use *
//     * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
//     * </p>
//     *
//     * @return whether this command has finished.
//     */
//    @Override
//    public boolean isFinished() {return isfinished;}
//
//
//    /**
//     * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
//     * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
//     * up loose ends, like shutting off a motor that was being used in the command.
//     *
//     * @param interrupted whether the command was interrupted/canceled
//     */
//    @Override
//    public void end(boolean interrupted)
//    {
//        button_count = 0;
//        exit_timer = 0;
//        algaeProcessor.stop();
//        isfinished = false;
//        deployed = false;
//    }
//
//    public void button_detect(){
//        button_count++;
//    }
//
////    public void return_button(){
////        SmartDashboard.putNumber("button presses", button_count);
////    }
//}
