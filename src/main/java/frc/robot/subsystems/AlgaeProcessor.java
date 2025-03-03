package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Units;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;

public class AlgaeProcessor extends SubsystemBase {
    private final SingleMotor wheelMotor;
    private final SingleMotor angleMotor;

    // processorPos is the current position of the processor encoder ticks.
    private final RelativeEncoder processorPos;
    private final DigitalInput tfLuna;
    private final PIDController deployPID;

    private double endPos;


    public AlgaeProcessor(DigitalInput tfLuna, SingleMotor wheelMotor, SingleMotor angleMotor, PIDController deployPID){
        this.wheelMotor = wheelMotor;
        this.angleMotor = angleMotor;
        this.tfLuna = tfLuna;
        this.deployPID = deployPID;

        this.processorPos = this.angleMotor.getRelativeEncoder();
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        SmartDashboard.putNumber("algae_processor_position", processorPos.getPosition());
        SmartDashboard.putNumber("algae_processor_angle", processorPos.getVelocity());
    }


    // This will be placed on a loop in RobotContainer.
    public void intakeAlgae() {
        if (!algaeDetected()){
            // Current voltage is 20, change in constants if needed.
            wheelMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_SWING_SPEED);
            // Current voltage is 10, change in constants if needed.
            angleMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_SPEED);
            endPos = getProcessorPos();
            processorPos.setPosition(0.0);
        } else {
            deployPID.setSetpoint(-endPos);
            wheelMotor.setVoltage(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_SPEED);
            angleMotor.accept(deployPID.calculate(getProcessorPos()));
            processorPos.setPosition(0.0);
        }
    }


    public void launchAlgae() {
        wheelMotor.setVoltage(-Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_INTAKE_SPEED);
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


    public void stopAngle() {
        angleMotor.stop();
    }


    public void stopIntake() {
        wheelMotor.stop();
    }


    public void resetEncoder() {
        processorPos.setPosition(0.0);
    }
}
