package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: Update sparkID, sensorID, climbVoltage, moveVoltage, check if motor requires voltage to lock
public class Climber extends SubsystemBase {
    private static int sparkID = 0;
    private static int sensorID = 0;
    private static int climbVoltage = 0;
    private static int moveVoltage = climbVoltage / 3;

    private SparkMax motor = new SparkMax(sparkID, SparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder1 = motor.getEncoder();
    private DigitalInput beamBreak = new DigitalInput(sensorID);


    public Command deployClimb(){
        if (!beamBreak.get()) {
            motor.setVoltage(moveVoltage);
        }
        return null; //idk how to do these
    }

    public Command stowClimb(){
        motor.setVoltage(-moveVoltage);
        return null; //idk how to do these
    }

    public Command engageClimb() {
        if (!beamBreak.get()) {
            motor.setVoltage(climbVoltage);
        } else if (beamBreak.get()){
            motor.stopMotor(); //only use if motor requires power while up
        } else {
            throw new IllegalStateException();
        }
        return null; //idk how to do these
    }

    public Command disengageClimb() {
        motor.setVoltage(-10);
        return null; //idk how to do these
    }

}

