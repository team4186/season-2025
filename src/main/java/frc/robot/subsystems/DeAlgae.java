package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import frc.robot.sparkmaxconfigs.Components;

public class DeAlgae extends SubsystemBase {

    private final SingleMotor wheelMotor = Components.getInstance().deAlgaeWheelMotor;
    private final SingleMotor angleMotor = Components.getInstance().deAlgaeAngleMotor;
    //private final SparkMax wheelMotor;
    //private final RelativeEncoder rollEncoder;
    //private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;


    public DeAlgae(){
        //wheelMotor = new SparkMax(CanId, MotorType.kBrushless);
        //rollEncoder = wheelMotor.getEncoder();

        //angleMotor = new SparkMax(CanId2, MotorType.kBrushless);
        angleEncoder = angleMotor.getLeadEncoder();
    }

    //TODO: find angle motor speed ratio
    public void runMotor_Up(){
        angleMotor.setSpeed(Constants.DeAlgaeConstants.speed/4);
        wheelMotor.setSpeed(Constants.DeAlgaeConstants.speed);
    }


    public void runMotor_Down(){
        angleMotor.setSpeed(-Constants.DeAlgaeConstants.speed/4);
        wheelMotor.setSpeed(-Constants.DeAlgaeConstants.speed);
    }


    public void deploy(){
        if(angleEncoder.getPosition() < Constants.DeAlgaeConstants.flatAngle){
            angleMotor.setSpeed(Constants.DeAlgaeConstants.speed/4);
        }
    }


    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // TODO: replace with pid or set to position within higher tolerance
    public void reset(){
        double tolerance = 0.5;
        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.armDefaultAngle - tolerance){
            angleMotor.setSpeed(Constants.DeAlgaeConstants.speed/4);
        } else if (angleEncoder.getPosition() > Constants.DeAlgaeConstants.armDefaultAngle + tolerance){
            angleMotor.setSpeed(-Constants.DeAlgaeConstants.speed/4);
        } else {
            stop();
        }
    }
}
