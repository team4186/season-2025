package frc.robot.sparkmaxconfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;


public class ElevatorMotorSet {
  private final SparkMax lead;


  public ElevatorMotorSet(SparkMax leader, SparkMax follower, SparkBaseConfig baseConfig, boolean inverted) {
    lead = leader;

    baseConfig
        .inverted(inverted)
        .openLoopRampRate(Constants.ElevatorConstants.ELEVATOR_RAMP_RATE);

    lead.configure(
        baseConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();

    followerConfig
        .apply(baseConfig)
        .follow(lead)
        .inverted( !inverted );

    follower.configure(
        followerConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }


  public RelativeEncoder getRelativeEncoder() {
    return lead.getEncoder();
  }


  public SparkMax getLead() {
    return lead;
  }


  public void setLeadVoltage(double voltage) {
    this.lead.setVoltage(voltage);
  }


  public void setLeadVoltage(Voltage voltage) {
    this.lead.setVoltage(voltage);
  }

  public void setLeadSpeed(double speed){this.lead.set(speed);}

  public void stop() {
    lead.stopMotor();
  }
}
