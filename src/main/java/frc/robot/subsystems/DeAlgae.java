package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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


    public DeAlgae(){
        //wheelMotor = new SparkMax(CanId, MotorType.kBrushless);
        //rollEncoder = wheelMotor.getEncoder();

        //angleMotor = new SparkMax(CanId2, MotorType.kBrushless);
        angleEncoder = angleMotor.getLeadEncoder();

        pid = new PIDController(
                Constants.DeAlgaeConstants.p,
                Constants.DeAlgaeConstants.i,
                Constants.DeAlgaeConstants.d);
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

    //TODO: PID implementation
    public void deploy(){
        double PIDoutput ;

        if(angleEncoder.getPosition() < Constants.DeAlgaeConstants.flatAngle){
            angleMotor.setSpeed(Constants.DeAlgaeConstants.speed/4);
        }

        PIDoutput = pid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.flatAngle);

        if (PIDoutput < Constants.DeAlgaeConstants.minSpeed){
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
//        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.armDefaultAngle - tolerance){
//            angleMotor.setSpeed(Constants.DeAlgaeConstants.speed/4);
//        } else if (angleEncoder.getPosition() > Constants.DeAlgaeConstants.armDefaultAngle + tolerance){
//            angleMotor.setSpeed(-Constants.DeAlgaeConstants.speed/4);
//        } else {
//            stop();
//        }
        PIDoutput = pid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.armDefaultAngle);

        if (PIDoutput < Constants.DeAlgaeConstants.minSpeed){
            angleMotor.stop();
        } else {
            angleMotor.setSpeed(PIDoutput);
        }



    }
}
