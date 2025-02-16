package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Components;


public class AlgaeProcessor extends SubsystemBase {
    // processorPos is the current position of the processor encoder ticks.
    private final double processorPos;

    // Placeholder CanID (Not actually 1).
    private static final int CANIDSwing = 1;
    // Placeholder CAN
    private static final int CANIDIntake = 2;
    // Change the input channel based on what is on the RoboRIO.

    private final DigitalInput TFLuna =  new DigitalInput(0);

    private final SparkMax swingMotor = new SparkMax(CANIDSwing, SparkLowLevel.MotorType.kBrushless);

    private final SparkMax intakeMotor = new SparkMax(CANIDIntake, SparkLowLevel.MotorType.kBrushless);

    public AlgaeProcessor(double processorPos) {
        this.processorPos = processorPos;
    }

    // This will be placed on a loop in RobotContainer.
    public Command intakeAlgaeCommand() {
        if (!algaeDetected()){
           // Current voltage is 20, change in constants if you want to.
            swingMotor.setVoltage(Components.AlgaeProcessorConstants.swingMotorVoltage);
            intakeMotor.setVoltage(Components.AlgaeProcessorConstants.intakeMotorVoltage);
        } else if (algaeDetected()) {

            // Make the intakeMotor continue to rotate capped at 10 Amps
            // so that the algae doesn't fall out.
        }
    }

    public Command launchAlgaeCommand() {
        // Should move the motors to launch algae.
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
        return processorPos;
    }
}
