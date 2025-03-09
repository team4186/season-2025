package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;
import frc.robot.UnitsUtility;

public class Climber extends SubsystemBase {

    private final DigitalInput limitSwitch;
    private final SingleMotor climberSingleMotor;
    private final RelativeEncoder angleEncoder;
    private final PIDController anglePid;
    private static double current_angle;
    private static double maxAngle, maxSpeed, minSpeed, defaultAngle;


    public Climber(SingleMotor climberSingleMotor, PIDController anglePid, DigitalInput limitSwitch){
        this.climberSingleMotor = climberSingleMotor;
        this.anglePid = anglePid;
        this.limitSwitch = limitSwitch;

        angleEncoder = climberSingleMotor.getRelativeEncoder();
        current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), "NEO550"));
        maxAngle = Constants.ClimberConstants.CLIMBER_MAX_ANGLE;
        maxSpeed = Constants.ClimberConstants.CLIMBER_MAX_SPEED;
        minSpeed = Constants.ClimberConstants.CLIMBER_MIN_SPEED;
        defaultAngle = Constants.ClimberConstants.CLIMBER_DEFAULT_ANGLE;
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        // SmartDashboard.putNumber("key", value);
        SmartDashboard.putNumber("Climber Angle:", getCurrentAngle());
        SmartDashboard.putNumber("Climber Speed:", getCurrentSpeed());
        SmartDashboard.putBoolean("Climber limitSwitch", UnitsUtility.isBeamBroken(limitSwitch,false,"Climber limit switch"));

    }


    //TODO: find angle motor speed ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor

    public void runMotor_Up(){
        current_angle = getCurrentAngle();
        double pidOutput = coerceIn(anglePid.calculate(current_angle,defaultAngle));

        climberSingleMotor.accept(pidOutput);
    }

    public double getCurrentAngle() {
        current_angle = (UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), "NEO550"));
        return current_angle;
    }

    public double getCurrentSpeed(){
        double angleSpeed = climberSingleMotor.motor.get();
        return angleSpeed;
    }


    // moves arm down with pid until it reaches the min angle while spinning the rolling motor inverted
    public void runMotor_Down(){
        current_angle = getCurrentAngle();
        if (UnitsUtility.isBeamBroken(limitSwitch,false,"Climber Limit Switch")) {
            climberSingleMotor.stop();
        } else {
            double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
            climberSingleMotor.accept(pidOutput);
        }
    }


    public void resetEncoder(){
        angleEncoder.setPosition(0.0);
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

    // stops the arm and rolling motors
    public void stop(){
        climberSingleMotor.stop();
    }

    // moves arm back to being parallel with the elevator with pid

    // this function returns, avoid using for now in favor of manReset function below
    public void reset(){
    }

    public void invertWheel(){
        climberSingleMotor.accept(-getCurrentSpeed());
    }
}
