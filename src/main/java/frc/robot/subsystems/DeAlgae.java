package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

public class DeAlgae {

    private SparkMax motor;
    private RelativeEncoder encoder;
    private static int CanId = 0; //TODO: placeHolder
    private double speed = 1.0; //TODO: placeHolder


    public DeAlgae(){
        motor = new SparkMax(CanId, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    public Command runMotor(boolean inverted){

        if(inverted){
            speed *= -1;
        }

        motor.set(speed);

        return null;
    }

    public Runnable stop(){

        motor.stopMotor();

        return null;
    }
}
