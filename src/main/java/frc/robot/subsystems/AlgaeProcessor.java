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
    private final SingleMotor wheelMotor = Components.getInstance().algaeProcessorMotor;
    private final SingleMotor angleMotor = Components.getInstance().algaeProcessorAngleMotor;

    // processorPos is the current position of the processor encoder ticks.
    private final RelativeEncoder processorPos = angleMotor.getEncoder();

    private final DigitalInput tfLuna = new DigitalInput(
            Constants.AlgaeProcessorConstants.lunaChannel);

    private final PIDController deployPID = new PIDController(
            Constants.AlgaeProcessorConstants.ALGAEPROCESSOR_P,
            Constants.AlgaeProcessorConstants.ALGAEPROCESSOR_I,
            Constants.AlgaeProcessorConstants.ALGAEPROCESSOR_D
    );

    private double endPos;



    // This will be placed on a loop in RobotContainer.
    public void intakeAlgaeCommand() {
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
            angleMotor.accept(deployPID.calculate(getProcessorPos()));
            processorPos.setPosition(0.0);
        }
    }


    public void launchAlgaeCommand() {
        wheelMotor.setVoltage(-Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
    }


    public boolean algaeDetected() {
        if (tfLuna.get()) {
            return true;
        } else if (!tfLuna.get()) {
            return false;
        } else {
            throw new IllegalStateException();
        }
    }

    public double getProcessorPos() {
        return Units.TicksToDegrees(processorPos.getPosition(), "NEO550");
    }
}
