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
    private static final int sparkID = 0;
    private static final int climbVoltage = 0;
    private static final int moveVoltage = climbVoltage / 3;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput beamBreak;


    public Climber( int beamBreakId ){
        beamBreak = new DigitalInput(beamBreakId);
        motor = new SparkMax(sparkID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }


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
        try {
            if (!beamBreak.get()) {
                motor.setVoltage(climbVoltage);
            } else {
                motor.stopMotor(); //only use if motor requires power while up
            }
        } catch (IllegalStateException e) {
            motor.stopMotor();
            String msg = "Beambreak error: " + e.toString();
            System.out.println( msg );
        }

        return null; //idk how to do these
    }


    // Different Functionality then Stow?
    public Command resetClimb() {
        motor.setVoltage(-10);
        return null; //idk how to do these
    }
}
