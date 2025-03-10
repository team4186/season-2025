package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    private static double maxAngle, maxSpeed, minSpeed;


    public Climber(SingleMotor climberSingleMotor, PIDController anglePid, DigitalInput limitSwitch){
        this.climberSingleMotor = climberSingleMotor;
        this.anglePid = anglePid;
        this.limitSwitch = limitSwitch;

        angleEncoder = climberSingleMotor.getRelativeEncoder();
        current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), "NEO550"));
        minSpeed = Constants.ClimberConstants.CLIMBER_MIN_SPEED;
        maxSpeed = Constants.ClimberConstants.CLIMBER_MAX_SPEED;
        maxAngle = Constants.ClimberConstants.CLIMBER_MAX_ANGLE;
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        // SmartDashboard.putNumber("key", value);
        SmartDashboard.putNumber("Climber Angle:", getCurrentAngle());
        SmartDashboard.putNumber("Climber Speed:", getCurrentSpeed());
        SmartDashboard.putBoolean("Climber limitSwitch", getBeamBreak());

    }

    private boolean getBeamBreak(){
        return !UnitsUtility.isBeamBroken(limitSwitch,false,"Climber limit switch");
    }


    //TODO: find angle motor speed ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor

    public void runMotor_Up(){

        if(!getBeamBreak()) {
            current_angle = getCurrentAngle();
            double pidOutput = coerceIn(anglePid.calculate(current_angle,maxAngle));
            climberSingleMotor.accept(pidOutput);
        }
        else{
            stop();
        }
    }


    public double getCurrentAngle() {
        current_angle = (UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), Constants.ClimberConstants.CLIMBER_GEARBOX_RATIO));
        return current_angle;
    }

    public double getCurrentSpeed(){
        double angleSpeed = climberSingleMotor.motor.get();
        return angleSpeed;
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


    public void coast(){
        SparkMaxConfig coastConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
        climberSingleMotor.motor.configure(coastConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void brake(){
        SparkMaxConfig brakeConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);
        climberSingleMotor.motor.configure(brakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
}
