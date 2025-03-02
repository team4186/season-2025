package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static double current_angle;
    private static double maxAngle, minAngle, maxSpeed, minSpeed, defaultAngle, flatAngle, wheelMaxSpeed, angleSpeed;


    public DeAlgae(SingleMotor wheelMotor, SingleMotor angleMotor){
        this.wheelMotor = wheelMotor;
        this.angleMotor = angleMotor;

        angleEncoder = angleMotor.getRelativeEncoder();

        current_angle = Math.toDegrees(Units.TicksToDegrees(angleEncoder.getPosition(), "NEO550"));
        maxAngle = Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE;
        minAngle = Constants.DeAlgaeConstants.DE_ALGAE_MIN_ANGLE;
        maxSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED;
        minSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MIN_SPEED;
        defaultAngle = Constants.DeAlgaeConstants.DE_ALGAE_DEFAULT_ANGLE;
        flatAngle = Constants.DeAlgaeConstants.DE_ALGAE_FLAT_ANGLE;

        wheelMaxSpeed = Constants.DeAlgaeConstants.DE_ALGAE_WHEEL_MAX_SPEED;
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("DeAlgae Angle:", getCurrentAngle());
        SmartDashboard.putNumber("DeAlgae Speed:", getCurrent_Speed());
    }


    //TODO: find angle motor speed ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor
    public void runMotor_Up(double upper_limit){
        current_angle = getCurrentAngle();
        if (current_angle < upper_limit) {

            angleMotor.setVoltage(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
        }
        else {
            angleMotor.stop();
        }
        wheelMotor.accept(-wheelMaxSpeed);
    }

    public boolean runMotor_Up(){
        current_angle = getCurrentAngle();

        wheelMotor.accept(-wheelMaxSpeed);

        if (current_angle < maxAngle) {

            angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
            return false;
        }

        else {
            angleMotor.stop();
            return true;
        }
    }

    public void ManrunMotor_Up(){
        current_angle = getCurrentAngle();
        if (current_angle < maxAngle) {

            angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
        }
        else {
            angleMotor.stop();
        }
        wheelMotor.accept(-wheelMaxSpeed);
    }

    public double getCurrentAngle() {
        current_angle = (Units.TicksToDegrees(angleEncoder.getPosition(), "NEO550"));
        return current_angle;
    }

    public double getCurrent_Speed(){
        angleSpeed = angleMotor.motor.get();
        return angleSpeed;
    }

    // moves arm down with pid until it reaches the min angle while spinning the rolling motor inverted
    public void runMotor_Down(){
        current_angle = getCurrentAngle();
        if (current_angle > minAngle){

            angleMotor.accept(-1*Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
        }
        else {
            angleMotor.stop();
        }

        wheelMotor.accept(wheelMaxSpeed);
    }


    public void resetEnconder(){
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


    //moves arm to be between two possible algae locations, estimated to be perpendicular to the elevator
    public boolean deploy() {
        double PIDoutput;
        current_angle = getCurrentAngle();

        if (current_angle >= flatAngle - 10.0 && current_angle <= flatAngle + 5.0) {
            angleMotor.stop();
            return true;

        }

        angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);

        return false;
    }

    //moves arm to be between two possible algae locations, estimated to be perpendicular to the elevator
    public void manDeploy() {
        double PIDoutput;
        current_angle = getCurrentAngle();

        if (current_angle >= flatAngle - 2.0 && current_angle <= flatAngle + 2.0) {
            angleMotor.stop();

        }
        angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);

    }

    // stops the arm and rolling motors
    public void stop(){
        wheelMotor.stop();
        angleMotor.stop();
    }


    // moves arm back to being parallel with the elevator with pid

    // this function returns, avoid using for now in favor of manReset function below
    public boolean reset(){
        double PIDoutput;
        current_angle = getCurrentAngle();

        if(current_angle > defaultAngle) {
            angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
            return false;
        }

        angleMotor.stop();
        return true;
    }

    public void manReset(){
        double PIDoutput;
        current_angle = getCurrentAngle();

        if(current_angle > defaultAngle) {
            angleMotor.accept(Constants.DeAlgaeConstants.DE_ALGAE_ANGLE_MOTOR_DEFAULT_VOLTAGE);
            return;
        }

        angleMotor.stop();
    }

    public void invertWheel(){
        wheelMotor.accept(-wheelMaxSpeed);
    }


    public void run_motor(){
        wheelMotor.accept(-wheelMaxSpeed);
    }
}
