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
    private static double resetAngle;

    public Climber(SingleMotor climberSingleMotor, DigitalInput limitSwitch){
        this.climberSingleMotor = climberSingleMotor;
        this.limitSwitch = limitSwitch;

        angleEncoder = climberSingleMotor.getRelativeEncoder();
        current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(),42, Constants.ClimberConstants.CLIMBER_GEARBOX_RATIO));
        deployVoltage = Constants.ClimberConstants.CLIMBER_DEPLOY_VOLTAGE;
        voltage = Constants.ClimberConstants.CLIMBER_VOLTAGE;
        deployAngle = Constants.ClimberConstants.CLIMBER_DEPLOY_ANGLE;
        resetAngle = Constants.ClimberConstants.CLIMBER_RESET_ANGLE;
        
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        // SmartDashboard.putNumber("key", value);
        SmartDashboard.putNumber("Climber_RelativeEncoder_Raw", angleEncoder.getPosition());
        SmartDashboard.putNumber("Climber_Angle:", getCurrentAngle());
        SmartDashboard.putNumber("Climber_Speed:", getCurrentSpeed());
        SmartDashboard.putBoolean("Climber_LimitSwitch", getBeamBreak());

    }

    private boolean getBeamBreak(){
        return !UnitsUtility.isBeamBroken(limitSwitch,false,"Climber limit switch");
    }


    //TODO: find angle motor voltage ratio
    //moves arm up with pid until it reaches the max angle while spinning the rolling motor

    public void deploy(){

        if(getCurrentAngle() <= deployAngle) {
            //Todo: Make sure (+/-) and direction is correct
            climberSingleMotor.setVoltage(deployVoltage);
        }
        else{
            climberSingleMotor.stop();
        }
    }

//    public boolean reset(){
//        if(Math.abs(getCurrentAngle()-resetAngle)<=10){
//            climberSingleMotor.stop();
//            return true;
//        } else if(getCurrentAngle()<=resetAngle){
//            climberSingleMotor.setVoltage(deployVoltage);
//            return false;
//        } else if(getCurrentAngle()>=resetAngle){
//            climberSingleMotor.setVoltage(-deployVoltage);
//            return false;
//        } else {
//            climberSingleMotor.stop();
//            return false;
//        }
//    }

    public boolean pull(){

        if(!getBeamBreak()) {
            climberSingleMotor.setVoltage(-voltage);
            return false;
        }
        else{
            climberSingleMotor.stop();
            return true;
        }
    }
    

    public double getCurrentAngle() {
        current_angle = (UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), 42, Constants.ClimberConstants.CLIMBER_GEARBOX_RATIO));
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
