package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeProcessor;


public class ProcessAlgaeCommand extends Command {
    private final AlgaeProcessor algaeProcessor;
    private boolean isFinished = false;

    public ProcessAlgaeCommand(AlgaeProcessor algaeProcessor) {
        this.algaeProcessor = algaeProcessor;
    }

    @Override
    public void initialize() {
        algaeProcessor.resetEncoder();
    }

    @Override
    public void execute() {
        algaeProcessor.intakeAlgae();
    }


    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // yeet
        algaeProcessor.launchAlgae();
    }
}
