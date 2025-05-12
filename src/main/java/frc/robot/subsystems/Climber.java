package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.SingleMotor;
import java.lang.Math;


public class Climber extends SubsystemBase {

  private final DigitalInput limitSwitch;
  private final SingleMotor climberSingleMotor;
  private final RelativeEncoder climbEncoder;
  private static double current_angle;
  private static double speed;
  private static double deployAngle;


  public Climber( SingleMotor climberSingleMotor, DigitalInput limitSwitch ) {
    this.climberSingleMotor = climberSingleMotor;
    this.limitSwitch = limitSwitch;

    climbEncoder = climberSingleMotor.getRelativeEncoder();
    climbEncoder.setPosition(0.0);

    current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(climbEncoder.getPosition(),Constants.ClimberConstants.CLIMBER_GEARBOX_RATIO));
    deployAngle = Constants.ClimberConstants.CLIMBER_DEPLOY_ANGLE;
    speed = Constants.ClimberConstants.CLIMBER_SPEED;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber_RelativeEncoder_Raw", getCurrentAngle());
    SmartDashboard.putNumber("Climber_Angle:", getCurrentAngle());
    SmartDashboard.putNumber("Climber_Speed:", getCurrentSpeed());
    SmartDashboard.putBoolean("Climber_LimitSwitch", getBeamBreak());

    if (getBeamBreak()) {
      resetEncoder();
    }
  }


  private boolean getBeamBreak() {
    return !UnitsUtility.isBeamBroken(limitSwitch,false,"Climber limit switch");
  }


  //TODO: find angle motor voltage ratio
  //moves arm up with pid until it reaches the max angle while spinning the rolling motor
  public void deploy() {

    if ( getCurrentAngle() <= deployAngle ) {
      //Todo: Make sure (+/-) and direction is correct
      climberSingleMotor.accept(speed);
    } else {
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


  public boolean pull() {

    if (!getBeamBreak()) {
      climberSingleMotor.accept(-speed);
      return false;
    } else {
      climberSingleMotor.stop();
      return true;
    }
  }


  public double getCurrentAngle() {
    current_angle = (UnitsUtility.ticksToDegrees(climbEncoder.getPosition(), Constants.ClimberConstants.CLIMBER_GEARBOX_RATIO));
    return current_angle;
  }


  public double getCurrentSpeed() {
    return climberSingleMotor.motor.get();
  }


  // stops the arm and rolling motors
  public void stop() {
    climberSingleMotor.stop();
  }


  public void coast() {
    SparkMaxConfig coastConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
    climberSingleMotor.motor.configure(coastConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }


  public void resetEncoder() {
    climbEncoder.setPosition(0.0);
  }
}
