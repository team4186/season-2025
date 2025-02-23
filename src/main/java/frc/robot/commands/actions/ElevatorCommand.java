//package frc.robot.commands.actions;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.Elevator;
//
//
//public final class ElevatorCommand extends Command {
//    // TODO: NOT a new elevator, you pass subsystem from robot container when calling command!
//    private final Elevator elevatorSubsystem;
//    private boolean isFinished = false;
//    private final int goalLevel;
//    private Task taskState;
//
//    private enum Task {
//        GO_TO_LEVEL,
//        RESET,
//        STOP
//    }
//
//
//    public ElevatorCommand(Elevator elevatorSubsystemParam, int requestedLevel) {
//        elevatorSubsystem = elevatorSubsystemParam;
//        goalLevel = requestedLevel;
//        taskState = Task.GO_TO_LEVEL;
//    }
//
//
//    /**
//     * The initial subroutine of a command.  Called once when the command is initially scheduled.
//     */
//    @Override
//    public void initialize() {
//        // Init stuff here.
//    }
//
//
//    /**
//     * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//     * until {@link #isFinished()}) returns true.)
//     */
//    @Override
//    public void execute() {
//
//        if (taskState == Task.GO_TO_LEVEL) {
//            // if break case -> taskState = Task.RESET
//            if(elevatorSubsystem.isAtLevelThreshold(goalLevel)){
//                taskState = Task.RESET;
//            }
//            // do thing to reach break point
//            elevatorSubsystem.goToLevel(goalLevel);
//        }
//
//        // Moved to level completed -> reset!
//        else if ( taskState == Task.RESET ) {
//            // if break case -> taskState = Task.STOP
//            if(elevatorSubsystem.getEncoderDistance() == 0.0){
//                taskState = Task.STOP;
//            }
//            elevatorSubsystem.reset();
//        }
//
//        // STOP case, Finish!
//        else if( taskState == Task.STOP){
//            //todo: IDK what this is, figure out
//            //isFinished = ( taskState.equals( Task.STOP ) );
//            elevatorSubsystem.stopMotor();
//        }
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
//    public boolean isFinished() {
//        return isFinished;
//    }
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
//    public void end(boolean interrupted) {
//        // Idle mode is set to brake. However, this will not hold the elevator in place.
//        // gravity will still continue to push the elevator down until the very bottom.
//        // However, brake does make the elevator descend slowly, so it wouldn't crash (this also resets encoders).
//        elevatorSubsystem.stopMotor();
//    }
//}
