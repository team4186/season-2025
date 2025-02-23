package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;
import frc.robot.Units;


public class DeAlgae extends SubsystemBase {

    private final SingleMotor wheelMotor;
    private final SingleMotor angleMotor;
    private final RelativeEncoder angleEncoder;
    private final PIDController anglePid;
    private static double current_angle;
    private static double maxAngle, minAngle, maxSpeed, minSpeed, defualtAngle, flatAngle;


    public DeAlgae(SingleMotor wheelMotor, SingleMotor angleMotor, PIDController anglePid){
        this.wheelMotor = wheelMotor;
        this.angleMotor = angleMotor;
        this.anglePid = anglePid;

        angleEncoder = angleMotor.getEncoder();

        current_angle = Units.TicksToDegrees(angleEncoder.getPosition(), "NEO550");
        maxAngle = Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE;
        minAngle = Constants.DeAlgaeConstants.DE_ALGAE_MIN_ANGLE;
        maxSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED;
        minSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MIN_SPEED;
        defualtAngle = Constants.DeAlgaeConstants.DE_ALGAE_DEFAULT_ANGLE;
        flatAngle = Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE;
    }


    //TODO: find angle motor speed ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor
    public void runMotor_Up(){
        if (current_angle < maxAngle) {
            double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
            angleMotor.accept(pidOutput);
        }

        wheelMotor.accept(maxSpeed);
    }


    // moves arm down with pid until it reaches the min angle while spinning the rolling motor inverted
    public void runMotor_Down(){
        if (current_angle > minAngle){
            double pidOutput = coerceIn(anglePid.calculate(current_angle,minAngle));
            angleMotor.accept(-pidOutput);
        }

        wheelMotor.accept(-maxSpeed);
    }

    // used to limit the pid calculation output to be within acceptable speeds
    private double coerceIn(double value) {
        int sign = 1;
        if (value < 0) {
            sign = -1;
        }

        if ( Math.abs( value ) > maxSpeed) {
            return maxSpeed * sign;
        } else {
            return Math.max( Math.abs(value), minSpeed) * sign;
        }
    }


    //moves arm to be between two possible algae locations, estimated to be perpendicular to the elevator
    public boolean deploy() {
        double PIDoutput;

        if (current_angle >= flatAngle) {
            return true;
        }

        PIDoutput = coerceIn(anglePid.calculate(current_angle, flatAngle));
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

        if(current_angle > defualtAngle) {
            PIDoutput = coerceIn(anglePid.calculate(current_angle, defualtAngle));
            angleMotor.accept(PIDoutput);
            return false;
        }

        return true;
    }
}
