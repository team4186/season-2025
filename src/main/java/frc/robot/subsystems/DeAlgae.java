package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sparkmaxconfigs.SingleMotor;
import frc.robot.sparkmaxconfigs.Components;
import edu.wpi.first.math.controller.PIDController;


public class DeAlgae extends SubsystemBase {

    private final SingleMotor wheelMotor = Components.getInstance().deAlgaeWheelMotor;
    private final SingleMotor angleMotor = Components.getInstance().deAlgaeAngleMotor;
    //private final SparkMax wheelMotor;
    //private final RelativeEncoder rollEncoder;
    //private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private PIDController pid;

    private static final int CanId = 0; //TODO: placeHolder
    private static final int CanId2 = 0; //TODO: placeHolder
    private static final double speed = 1.0; //TODO: placeHolder
    private static final double minSpeed = 0.5; //TODO: placeHolder
    private static final double armDefaultAngle = 0.0; //TODO: find arm offset
    private static final double flatAngle = 0.0; //TODO: find the 'distance' of 90 degrees
    private static double p = 0.5 , i = 0.0, d = 0.0; //TODO: tune pid values


    public DeAlgae(){
        //wheelMotor = new SparkMax(CanId, MotorType.kBrushless);
        //rollEncoder = wheelMotor.getEncoder();

        //angleMotor = new SparkMax(CanId2, MotorType.kBrushless);
        angleEncoder = angleMotor.getLeadEncoder();

        pid = new PIDController(p,i,d);
    }

    //TODO: find angle motor speed ratio
    public void runMotor_Up(){
        angleMotor.setSpeed(speed/4);
        wheelMotor.setSpeed(speed);
    }


    public void runMotor_Down(){
        angleMotor.setSpeed(-speed/4);
        wheelMotor.setSpeed(-speed);
    }

    //TODO: PID implementation
    public void deploy(){
        double PIDoutput ;

        if(angleEncoder.getPosition() < flatAngle){
            angleMotor.setSpeed(speed/4);
        }

        PIDoutput = pid.calculate(angleEncoder.getPosition(),flatAngle);

        if (PIDoutput < minSpeed){
            angleMotor.stop();
        }
        else{
            angleMotor.setSpeed(PIDoutput);
        }
    }


    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // TODO: replace with pid or set to position within higher tolerance // done, needs testing
    public void reset(){
        double PIDoutput;

//        double tolerance = 0.5;
//        if (angleEncoder.getPosition() < armDefaultAngle - tolerance){
//            angleMotor.setSpeed(speed/4);
//        } else if (angleEncoder.getPosition() > armDefaultAngle + tolerance){
//            angleMotor.setSpeed(-speed/4);
//        } else {
//            stop();
//        }

        PIDoutput = pid.calculate(angleEncoder.getPosition(),armDefaultAngle);

        if (PIDoutput < minSpeed){
            angleMotor.stop();
        }
        else{
            angleMotor.setSpeed(PIDoutput);
        }



    }
}
