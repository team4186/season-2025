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
    private final PIDController deployPID = new PIDController(Constants.PIDConstants.proportional, Constants.PIDConstants.integral, Constants.PIDConstants.derivative);

    private final SingleMotor algaeMotor = Components.getInstance().algaeProcessorMotor;

    private final SingleMotor deployMotor = Components.getInstance().processorDeployMotor;

    // processorPos is the current position of the processor encoder ticks.
    private final RelativeEncoder processorPos = deployMotor.motor.getAlternateEncoder();

    private double endPos;

    // Change the input channel based on what is on the RoboRIO.
    private final DigitalInput TFLuna =  new DigitalInput(0);

    // This will be placed on a loop in RobotContainer.
    public Command intakeAlgaeCommand() {
        if (!algaeDetected()){
            // Current voltage is 20, change in constants if needed.
            algaeMotor.setVoltage(Constants.SubsystemMotorConstants.ALGAE_PROCESSOR_SWING_VOLTAGE);
            // Current voltage is 10, change in constants if needed.
            deployMotor.setVoltage(Constants.SubsystemMotorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
            this.endPos = getProcessorPos();
            processorPos.setPosition(0.0);
        } else if (algaeDetected()) {
            deployPID.setSetpoint(-endPos);
            algaeMotor.setVoltage(-Constants.SubsystemMotorConstants.ALGAE_PROCESSOR_INTAKE_VOLTAGE);
            deployMotor.setSpeed(deployPID.calculate(getProcessorPos()));
        }
        // TODO: Fix this later.
        return null;
    }

    public Command launchAlgaeCommand() {
        // Should move the motors to launch algae.

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
