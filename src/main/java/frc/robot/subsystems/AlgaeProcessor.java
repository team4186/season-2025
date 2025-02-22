package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Units;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;

public class AlgaeProcessor extends SubsystemBase {
    private final PIDController deployPID = new PIDController(
            Constants.AlgaeProcessorConstants.proportional,
            Constants.AlgaeProcessorConstants.integral,
            Constants.AlgaeProcessorConstants.derivative);

    private final SingleMotor wheelMotor = Components.getInstance().algaeProcessorMotor;

    private final SingleMotor angleMotor = Components.getInstance().algaeProcessorAngleMotor;

    // processorPos is the current position of the processor encoder ticks.
    private final RelativeEncoder processorPos = angleMotor.getEncoder();

    private double endPos;

    // Change the input channel based on what is on the RoboRIO.
    private final DigitalInput TFLuna =  new DigitalInput(0);

    // This will be placed on a loop in RobotContainer.
    public Command intakeAlgaeCommand() {
        if (!algaeDetected()){
            // Current voltage is 20, change in constants if needed.
            wheelMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_SWING_VOLTAGE);
            // Current voltage is 10, change in constants if needed.
            angleMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
            this.endPos = getProcessorPos();
            processorPos.setPosition(0.0);
        } else if (algaeDetected()) {
            deployPID.setSetpoint(-endPos);
            wheelMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
            angleMotor.setSpeed(deployPID.calculate(getProcessorPos()));
            processorPos.setPosition(0.0);
        }
        // TODO: Fix this later.
        return null;
    }

    public Command launchAlgaeCommand() {
        wheelMotor.setVoltage(-Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
        return null;
    }

    public boolean algaeDetected() {
        if (TFLuna.get()) {
            return true;
        } else if (!TFLuna.get()) {
            return false;
        } else {
            throw new IllegalStateException();
        }
    }

    public double getProcessorPos() {
        return Units.TicksToDegrees(processorPos.getPosition(), "NEO550");
    }
}
