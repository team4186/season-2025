package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;
import frc.robot.Units;

public class AlgaeProcessor extends SubsystemBase {

    private final SingleMotor wheelMotor;
    private final SingleMotor angleMotor;
    private final RelativeEncoder angleEncoder;
    private final PIDController anglePid;
    private static double current_angle;
    private static double maxAngle, minAngle, maxSpeed, minSpeed, defaultAngle, flatAngle, wheelMaxSpeed, angleSpeed, deployAngle;


    public AlgaeProcessor(SingleMotor wheelMotor, SingleMotor angleMotor, PIDController anglePid){
        this.wheelMotor = wheelMotor;
        this.angleMotor = angleMotor;
        this.anglePid = anglePid;

        angleEncoder = angleMotor.getRelativeEncoder();

        current_angle = Math.toDegrees(Units.TicksToDegrees(angleEncoder.getPosition(), "NEO550"));
        maxAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MAX_ANGLE;
        minAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MIN_ANGLE;
        maxSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MAX_SPEED;
        minSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MIN_SPEED;
        defaultAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_DEFAULT_ANGLE;
        deployAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_DEPLOY_ANGLE;

        wheelMaxSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_WHEEL_MAX_SPEED;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Processor Angle: ", getCurrentAngle());
        SmartDashboard.putNumber("Processor Speed: ", getCurrentSpeed());
    }


    //TODO: find angle motor speed ratio

    public boolean deploy(){
        current_angle = getCurrentAngle();

        wheelMotor.accept(-wheelMaxSpeed);

        if (current_angle < maxAngle) {

            double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
            angleMotor.accept(pidOutput);
            return false;
        }

        else {
            angleMotor.stop();
            return true;
        }
    }

    public double getCurrentAngle() {
        current_angle = (Units.TicksToDegrees(angleEncoder.getPosition(), "NEO550"));
        return current_angle;
    }

    public double getCurrentSpeed(){
        angleSpeed = angleMotor.motor.get();
        return angleSpeed;
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
        wheelMotor.stop();
        angleMotor.stop();
    }

    // moves arm back to being parallel with the elevator with pid

    // this function returns, avoid using for now in favor of manReset function below
    public boolean reset(){
        double PIDoutput;
        current_angle = getCurrentAngle();

        if(current_angle > defaultAngle) {
            PIDoutput = coerceIn(anglePid.calculate(current_angle, defaultAngle));
            angleMotor.accept(PIDoutput);
            return false;
        }

        angleMotor.stop();
        return true;
    }
}
