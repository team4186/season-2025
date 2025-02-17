package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeAlgae extends SubsystemBase {

    private final SparkMax wheelMotor;
    private final RelativeEncoder rollEncoder;
    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;

    private static final int CanId = 0; //TODO: placeHolder
    private static final int CanId2 = 0; //TODO: placeHolder
    private static final double speed = 1.0; //TODO: placeHolder
    private static final double armDefaultAngle = 0.0; //TODO: find arm offset
    private static final double flatAngle = 0.0; //TODO: find the 'distance' of 90 degrees


    public DeAlgae(){
        wheelMotor = new SparkMax(CanId, MotorType.kBrushless);
        rollEncoder = wheelMotor.getEncoder();

        angleMotor = new SparkMax(CanId2, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
    }

    //TODO: find angle motor speed ratio
    public void runMotor_Up(){
        angleMotor.set(speed/4);
        wheelMotor.set(speed);
    }


    public void runMotor_Down(){
        angleMotor.set(-speed/4);
        wheelMotor.set(-speed);
    }


    public void deploy(){
        if(angleEncoder.getPosition() < flatAngle){
            angleMotor.set(speed/4);
        }
    }


    public void stop(){
        wheelMotor.stopMotor();
        angleMotor.stopMotor();
    }


    // TODO: replace with pid or set to position within higher tolerance
    public void reset(){
        double tolerance = 0.5;
        if (angleEncoder.getPosition() < armDefaultAngle - tolerance){
            angleMotor.set(speed/4);
        } else if (angleEncoder.getPosition() > armDefaultAngle + tolerance){
            angleMotor.set(-speed/4);
        } else {
            stop();
        }
    }
}
