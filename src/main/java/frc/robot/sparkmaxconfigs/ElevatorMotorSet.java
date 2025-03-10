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


    public ElevatorMotorSet(SparkMax lead, SparkMax follower, SparkBaseConfig baseConfig, boolean inverted) {
        baseConfig
                .inverted( inverted )
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

        this.lead = lead;
    }


    public RelativeEncoder getRelativeEncoder(){
        return this.lead.getEncoder();
    }

    public SparkMax getLead(){
        return this.lead;
    }

    public void setLeadVoltage(double voltage) { this.lead.setVoltage( voltage ); }

    public void setLeadVoltage(Voltage voltage) { this.lead.setVoltage(voltage); }


    public void stop(){
        this.lead.stopMotor();
    }
}
