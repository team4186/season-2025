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
    private static double current_angle;
    private static double deployVoltage;
    private static double voltage;
    private static double deployAngle;


    public Climber(SingleMotor climberSingleMotor, DigitalInput limitSwitch){
        this.climberSingleMotor = climberSingleMotor;
        this.limitSwitch = limitSwitch;

        angleEncoder = climberSingleMotor.getRelativeEncoder();
        current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), "NEO550"));
        deployVoltage = Constants.ClimberConstants.CLIMBER_DEPLOY_VOLTAGE;
        voltage = Constants.ClimberConstants.CLIMBER_VOLTAGE;
        deployAngle = Constants.ClimberConstants.CLIMBER_DEPLOY_ANGLE;
        
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


    //TODO: find angle motor voltage ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor

    public void deploy(){

        if(getCurrentAngle() <= deployAngle) {
            climberSingleMotor.setVoltage(deployVoltage);
        }
        else{
            climberSingleMotor.stop();
        }
    }

    public boolean stow(){

        if(!getBeamBreak()) {
            climberSingleMotor.setVoltage(-deployVoltage);
            return false;
        }
        else{
            climberSingleMotor.stop();
            return true;
        }
    }

    public void pull(){

        if(!getBeamBreak()) {
            climberSingleMotor.setVoltage(voltage);
        }
        else{
            climberSingleMotor.stop();
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


    // stops the arm and rolling motors
    public void stop(){
        climberSingleMotor.stop();
    }
}
