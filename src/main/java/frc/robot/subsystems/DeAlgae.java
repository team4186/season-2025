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
    public void runMotor_Up(){
        if (angleEncoder.getPosition() < Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE) {
            double pidOutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE));
            angleMotor.accept(pidOutput);
        }

        wheelMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED);
    }


    // TODO: not sure about pid behavior when given position is more than target position
    public void runMotor_Down(){
        if (angleEncoder.getPosition() > Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE){
            double pidOutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(),Constants.DeAlgaeConstants.DE_ALGAE_MIN_ANGLE));
            angleMotor.accept(-pidOutput);
        }

        wheelMotor.accept(-Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED);
    }


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


    //TODO: PID implementation
    public boolean deploy() {
        double PIDoutput;

        if (angleEncoder.getPosition() >= Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE) {
            return true;
        }

        PIDoutput = coerceIn(anglePid.calculate(angleEncoder.getPosition(), Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE));
        angleMotor.accept(PIDoutput);

        return false;
    }


    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // TODO: replace with pid or set to position within higher tolerance // done, needs testing
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
