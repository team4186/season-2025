package frc.robot.sparkmaxconfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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

    public void accept(double value) {
        this.lead.set(value);
    }


    public void stop(){
        this.lead.stopMotor();
    }
}
