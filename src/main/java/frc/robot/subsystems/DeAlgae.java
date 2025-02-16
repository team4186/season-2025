package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units;

public class DeAlgae extends SubsystemBase {

    private final SparkMax rollMotor;
    private final RelativeEncoder rollEncoder;
    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;

    private static int CanId = 0; //TODO: placeHolder
    private static double speed = 1.0; //TODO: placeHolder
    private static double armDefaultAngle = 0.0; //TODO: find arm offset
    private static double flatAngle = 0.0; //TODO: find the 'distance' of 90 degrees


    public DeAlgae(){
        rollMotor = new SparkMax(CanId, MotorType.kBrushless);
        rollEncoder = rollMotor.getEncoder();

        angleMotor = new SparkMax(CanId, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
    }


    public void runMotor_Up(){
        angleMotor.set(speed/4);
        rollMotor.set(speed);
    }

    public void runMotor_inverted_Down(){
        angleMotor.set(-speed/4);
        rollMotor.set(-speed);
    }

    public void stickOut(){
        if(angleEncoder.getPosition() < flatAngle){
            angleMotor.set(speed/4);
        }
    }

    public void stop(){
        rollMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public void reset(){
        if(angleEncoder.getPosition() < armDefaultAngle){
            angleMotor.set(speed/4);
        }
        else if(angleEncoder.getPosition() > armDefaultAngle){
            angleMotor.set(-speed/4);
        }
        else{
            stop();
        }
    }

}
