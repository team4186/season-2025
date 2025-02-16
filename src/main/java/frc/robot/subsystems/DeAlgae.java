package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DeAlgae {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private static int CanId = 0; //TODO: placeHolder
    private static double runTime = 5.0;
    private double speed = 1.0;


    public DeAlgae(){
        motor = new SparkMax(CanId, MotorType.kBrushless);

        encoder = motor.getEncoder();
    }

    public void runMotor(boolean inverted){
        double ticks = 0.0;

        if(inverted){
            speed *= -1;
        }

        while (ticks < runTime) {
            motor.set(speed);
            ticks++;
        }

        motor.stopMotor();
    }
}
