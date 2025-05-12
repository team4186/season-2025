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


public class DeAlgae extends SubsystemBase {

  private final DigitalInput limitSwitch;
  private final SingleMotor wheelMotor;
  private final SingleMotor angleMotor;
  private final RelativeEncoder angleEncoder;
  private final PIDController anglePid;
  private static double current_angle;
  private static double maxAngle, maxSpeed, minSpeed, defaultAngle, wheelMaxSpeed, angleSpeed;


  public DeAlgae( SingleMotor wheelMotor, SingleMotor angleMotor, PIDController anglePid, DigitalInput limitSwitch ) {
    this.wheelMotor = wheelMotor;
    this.angleMotor = angleMotor;
    this.anglePid = anglePid;
    this.limitSwitch = limitSwitch;

    angleEncoder = angleMotor.getRelativeEncoder();
    current_angle = Math.toDegrees(UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), Constants.DeAlgaeConstants.DE_ALGAE_GEARBOX_RATIO));
    maxAngle = Constants.DeAlgaeConstants.DE_ALGAE_MAX_ANGLE;
    maxSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MAX_SPEED;
    minSpeed = Constants.DeAlgaeConstants.DE_ALGAE_MIN_SPEED;
    defaultAngle = Constants.DeAlgaeConstants.DE_ALGAE_DEFAULT_ANGLE;
    wheelMaxSpeed = Constants.DeAlgaeConstants.DE_ALGAE_WHEEL_MAX_SPEED;
  }


  @Override
  public void periodic() {
    // publish smart dashboard info here
    // SmartDashboard.putNumber("key", value);
    SmartDashboard.putNumber("DeAlgae_Angle:", getCurrentAngle());
    SmartDashboard.putNumber("DeAlgae_Speed:", getCurrentSpeed());
    SmartDashboard.putBoolean("DeAlgae_LimitSwitch", getBeamBreak());
    SmartDashboard.putNumber("DeAlgae_RelativeEncoder_Raw", angleEncoder.getPosition());

    if (getBeamBreak()) {
      resetEncoder();
    }
  }


  private boolean getBeamBreak() {
    return !UnitsUtility.isBeamBroken(limitSwitch,false,"DeAlgae limit switch");
  }


  // TODO: find angle motor speed ratio
  // moves arm up with pid until it reaches the max angle while spinning the rolling motor
  public boolean pid_runMotor_Up() {
    current_angle = getCurrentAngle();

    wheelMotor.accept(-wheelMaxSpeed);

    double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
    angleMotor.accept(pidOutput);

    return current_angle >= maxAngle - 2 || current_angle <= maxAngle + 2;
  }


  public void Manpid_runMotor_Up() {
    current_angle = getCurrentAngle();

    wheelMotor.accept(-wheelMaxSpeed);

    double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
    angleMotor.accept(pidOutput);
  }


  public void runMotor_Up( double upper_limit ) {
    current_angle = getCurrentAngle();
    if (current_angle < upper_limit) {

      double pidOutput = coerceIn(anglePid.calculate(current_angle, upper_limit));
      angleMotor.accept(pidOutput);
    } else {
      angleMotor.stop();
    }

    wheelMotor.accept(-wheelMaxSpeed);
  }


  public boolean runMotor_Up() {
    current_angle = getCurrentAngle();

    wheelMotor.accept(-wheelMaxSpeed);

    if (current_angle < maxAngle) {
      angleMotor.accept(maxSpeed);
      return false;
    } else {
      angleMotor.stop();
      return true;
    }
  }


  public void manRunMotor_Up() {
    current_angle = getCurrentAngle();

    if (current_angle < maxAngle) {
      double pidOutput = coerceIn(anglePid.calculate(current_angle, maxAngle));
      angleMotor.accept(pidOutput);
    } else {
      angleMotor.stop();
    }

    wheelMotor.accept(-wheelMaxSpeed);
  }


  public double getCurrentAngle() {
    current_angle = (UnitsUtility.ticksToDegrees(angleEncoder.getPosition(), Constants.DeAlgaeConstants.DE_ALGAE_GEARBOX_RATIO));
    return current_angle;
  }


  public double getCurrentSpeed() {
    angleSpeed = angleMotor.motor.get();
    return angleSpeed;
  }


  public void resetEncoder(){
    angleEncoder.setPosition(0.0);
  }


  // used to limit the pid calculation output to be within acceptable speeds
  private double coerceIn( double value ) {
    int sign = 1;
    if (value < 0) {
      sign = -1;
    }

    if ( Math.abs( value ) > maxSpeed ) {
      return maxSpeed * sign;
    } else {
      return Math.max( Math.abs(value), minSpeed) * sign;
    }
  }


  // stops the arm and rolling motors
  public void stop() {
    wheelMotor.stop();
    angleMotor.stop();
  }


  // moves arm back to being parallel with the elevator with pid
  // this function returns, avoid using for now in favor of manReset function below
  public boolean reset() {
    double PIDoutput;
    current_angle = getCurrentAngle();

    if (current_angle > defaultAngle) {
      angleMotor.accept(-maxSpeed);
      return false;
    }

    angleMotor.stop();
    return true;
  }


  public boolean pid_reset() {
    double PIDoutput;
    current_angle = getCurrentAngle();

    if (current_angle >= defaultAngle -1 || current_angle <= defaultAngle + 1 || getBeamBreak()) {
      stop();
      resetEncoder();
      return true;
    }

    PIDoutput = coerceIn(anglePid.calculate(current_angle, defaultAngle));
    angleMotor.accept(PIDoutput);
    return false;

  }


  public void manPid_reset() {
    double PIDoutput;
    current_angle = getCurrentAngle();

    if (!getBeamBreak()) {
      PIDoutput = coerceIn(anglePid.calculate(current_angle, defaultAngle));
      angleMotor.accept(PIDoutput);
    } else {
      stop();
      resetEncoder();
    }

  }


  public void manReset() {
    double PIDoutput;
    current_angle = getCurrentAngle();

    if (current_angle > defaultAngle || !getBeamBreak()) {
      PIDoutput = coerceIn(anglePid.calculate(current_angle, defaultAngle));
      angleMotor.accept(PIDoutput);
      return;
    }

    angleMotor.stop();
    resetEncoder();
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
