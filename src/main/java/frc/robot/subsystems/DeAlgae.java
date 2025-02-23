package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;


public class DeAlgae extends SubsystemBase {

    private final SingleMotor wheelMotor;
    private final SingleMotor angleMotor;
    private final RelativeEncoder angleEncoder;
    private final PIDController anglePid;


    public DeAlgae(SingleMotor wheelMotor, SingleMotor angleMotor, PIDController anglePid){
        this.wheelMotor = wheelMotor;
        this.angleMotor = angleMotor;
        this.anglePid = anglePid;

        angleEncoder = angleMotor.getEncoder();
    }


    //TODO: find angle motor speed ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor
    public void runMotor_Up(){
        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE) {
            double pidOutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE));
            angleMotor.accept(pidOutput);
        }

        wheelMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED);
    }


    // moves arm down with pid until it reaches the min angle while spinning the rolling motor inverted
    public void runMotor_Down(){
        if (angleEncoder.getPosition() > Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE){
            double pidOutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.DE_ALGAE_MIN_ANGLE));
            angleMotor.accept(-pidOutput);
        }

        wheelMotor.accept(-Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED);
    }

    // used to limit the pid calculation output to be within acceptable speeds
    private double coerceIn(double value) {
        int sign = 1;
        if (value < 0) {
            sign = -1;
        }

        if ( Math.abs( value ) > Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED) {
            return Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED * sign;
        } else {
            return Math.max( Math.abs(value), Constants.DeAlgaeConstants.DE_ALGAE_MIN_SPEED) * sign;
        }
    }


    //moves arm to be between two possible algae locations, estimated to be perpendicular to the elevator
    public boolean deploy() {
        double PIDoutput;

        if (angleEncoder.getPosition() >= Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE) {
            return true;
        }

        PIDoutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE));
        angleMotor.accept(PIDoutput);

        return false;
    }

    // stops the arm and rolling motors
    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // moves arm back to being parallel with the elevator with pid
    public boolean reset(){
        double PIDoutput;

        if(angleEncoder.getPosition() > Constants.DeAlgaeConstants.DE_ALGAE_DEFAULT_ANGLE) {
            PIDoutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.DE_ALGAE_DEFAULT_ANGLE));
            angleMotor.accept(PIDoutput);
            return false;
        }

        return true;
    }
}
