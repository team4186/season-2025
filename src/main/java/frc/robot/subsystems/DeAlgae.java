package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeAlgae extends SubsystemBase {

    private SparkMax motor;
    private RelativeEncoder encoder;
    private static int CanId = 0; //TODO: placeHolder
    private double speed = 1.0; //TODO: placeHolder


    public DeAlgae(){
        motor = new SparkMax(CanId, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }


    public void runMotor(){
        motor.set(speed);
    }

    public void runMotor_inverted(){
        motor.set(-speed);
    }

    public void stop(){
        motor.stopMotor();
    }
}
