package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorFailsafeCommand extends Command {
    private final Elevator elevatorSubsystem;
    private final double speed;
    private boolean isFinished;
    public ElevatorFailsafeCommand(Elevator elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        isFinished=false;
    }

    @Override
    public void execute(){
        if (!elevatorSubsystem.isAtBottom()) {
            elevatorSubsystem.elevatorZeroFailsafe(speed);
        }
        else{
            isFinished=true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }


}
