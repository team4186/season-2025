package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.SingleMotor;
import java.lang.Math;


public class AlgaeProcessor extends SubsystemBase {

  private final DigitalInput limitSwitch;
  private final SingleMotor wheelMotor;
  private final SingleMotor angleMotor;
  private final RelativeEncoder angleEncoder;
  private static double current_angle;
  private static double maxAngle, maxSpeed, defaultAngle, holdingAngle,
      wheelIntakeSpeed, wheelOutputSpeed, angleSpeed, holdingSpeed;


  public AlgaeProcessor(SingleMotor wheelMotor, SingleMotor angleMotor, DigitalInput limitSwitch) {
    this.wheelMotor = wheelMotor;
    this.angleMotor = angleMotor;
    this.limitSwitch = limitSwitch;

    angleEncoder = angleMotor.getRelativeEncoder();
    current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_GEARBOX_RATIO));
    maxAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MAX_ANGLE;
    maxSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_MAX_SPEED;
    defaultAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_DEFAULT_ANGLE;
    holdingAngle = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_HOLDING_ANGLE;
    wheelIntakeSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_WHEEL_INTAKE_SPEED;
    wheelOutputSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_WHEEL_OUTPUT_SPEED;
    holdingSpeed = Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_WHEEL_HOLDING_SPEED;
  }


  @Override
  public void periodic() {
    // publish smart dashboard info here
    // SmartDashboard.putNumber("key", value);
    SmartDashboard.putNumber("Algae_Processor_Relative_Encoder_Raw", angleEncoder.getPosition());
    SmartDashboard.putNumber("Algae_Processor_Angle:", getCurrentAngle());
    SmartDashboard.putNumber("Algae_Processor_Speed:", getCurrentSpeed());
    SmartDashboard.putBoolean("Algae_Processor_limitSwitch", !UnitsUtility.isBeamBroken(limitSwitch,false,"Algae processor limit switch"));
    if (getBeamBreak()) {
      resetEncoder();
    }
  }


  private boolean getBeamBreak() {
    return !UnitsUtility.isBeamBroken(limitSwitch,false,"Processor limit switch");
  }


  //TODO: find angle motor speed ratio
  //moves arm up with pid until it reaches the max angle while spinning the rolling motor
  public void runMotor_Up() {
    current_angle = getCurrentAngle();
    //Todo: Check Inverse, update constants
    if (!getBeamBreak() && current_angle > holdingAngle) {
      angleMotor.accept(-maxSpeed);
    } else {
      stop();
    }
  }


  public boolean cmd_runMotor_Up() {
    current_angle = getCurrentAngle();
    //Todo: Check Inverse, update constants
    if (!getBeamBreak() && current_angle > holdingAngle) {
      angleMotor.accept(-maxSpeed);
      return false;
    }

    stop();
    return true;
  }


  public void runMotor_Down() {
    current_angle = getCurrentAngle();
    //Todo: Check Inverse, update constants
    if (current_angle < maxAngle) {
      angleMotor.accept(maxSpeed);
    } else {
      stop();
    }
  }


  public void cmd_runMotor_Down() {
    current_angle = getCurrentAngle();
    //Todo: Check Inverse, update constants
    if (current_angle < maxAngle) {
      angleMotor.accept(maxSpeed);
    } else {
      stop();
    }
  }


  public double getCurrentAngle() {
    current_angle = (UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_GEARBOX_RATIO));
    return current_angle;
  }


  public double getCurrentSpeed() {
    angleSpeed = angleMotor.motor.get();
    return angleSpeed;
  }


  // moves arm down with pid until it reaches the min angle while spinning the rolling motor inverted
  public void resetEncoder() {
    angleEncoder.setPosition(0.0);
  }


  // stops the arm and rolling motors
  public void stop() {
    wheelMotor.stop();
    angleMotor.stop();
  }


  public void wheelStop() {
    wheelMotor.stop();
  }


  // moves arm back to being parallel with the elevator with pid
  // this function returns, avoid using for now in favor of manReset function below
  public boolean reset(){
    current_angle = getCurrentAngle();

    if (!getBeamBreak() || current_angle > defaultAngle) {
      angleMotor.accept(-maxSpeed);
      return false;
    }

    angleMotor.stop();
    return true;
  }


  public void eject() {
    wheelMotor.accept(wheelOutputSpeed);
  }


  public void intake() {
    wheelMotor.accept(-wheelIntakeSpeed);
  }


  public void holdAlgae() {
    wheelMotor.accept(-holdingSpeed);
  }


  public void coast() {
    SparkMaxConfig coastConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
    angleMotor.motor.configure(coastConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }


  public void brake() {
    SparkMaxConfig brakeConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);
    angleMotor.motor.configure(brakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
}
