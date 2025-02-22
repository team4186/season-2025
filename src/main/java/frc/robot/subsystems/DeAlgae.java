package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import frc.robot.sparkmaxconfigs.Components;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;


public class DeAlgae extends SubsystemBase {

    private final SingleMotor wheelMotor = Components.getInstance().deAlgaeWheelMotor;
    private final SingleMotor angleMotor = Components.getInstance().deAlgaeAngleMotor;
    private final RelativeEncoder angleEncoder;
    private PIDController pid;


    public DeAlgae(){
        angleEncoder = angleMotor.getEncoder();

        pid = new PIDController(
                Constants.DeAlgaeConstants.p,
                Constants.DeAlgaeConstants.i,
                Constants.DeAlgaeConstants.d);
    }

    //TODO: find angle motor speed ratio
    public void runMotor_Up(){
        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.maxAngle) {
            double pidOutput = coerceIn(pid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.maxAngle));
            angleMotor.setSpeed(pidOutput);
        }

        wheelMotor.setSpeed(Constants.DeAlgaeConstants.topSpeed);
    }


    public void runMotor_Down(){
        if (angleEncoder.getPosition() > Constants.DeAlgaeConstants.minAngle){
            double pidOutput = coerceIn(pid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.minAngle));
            angleMotor.setSpeed(pidOutput);
        }

        wheelMotor.setSpeed(-Constants.DeAlgaeConstants.topSpeed);
    }


    private double coerceIn(double value) {
        if (abs(value) > Constants.DeAlgaeConstants.topSpeed) {
            if(value > 0) {return Constants.DeAlgaeConstants.topSpeed;}

            else{return -Constants.DeAlgaeConstants.topSpeed;}

        } 
        else if (abs(value) < Constants.DeAlgaeConstants.minSpeed) {
            if(value > 0){return Constants.DeAlgaeConstants.minSpeed;}

            else{return Constants.DeAlgaeConstants.minSpeed;}
        } 
        else {return value;}
    }


    public boolean deploy() {
        double PIDoutput;

        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.flatAngle) {
            PIDoutput = coerceIn(pid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.flatAngle));

            angleMotor.setSpeed(PIDoutput);
        }
        else {
            return true;
        }

        return false;
    }

    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // TODO: replace with pid or set to position within higher tolerance // done, needs testing
    public boolean reset(){
        double PIDoutput;

        if(angleEncoder.getPosition() > Constants.DeAlgaeConstants.armDefaultAngle) {
            PIDoutput = coerceIn(pid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.armDefaultAngle));
            angleMotor.setSpeed(PIDoutput);

            return false;
        }

        return true;
    }
}

