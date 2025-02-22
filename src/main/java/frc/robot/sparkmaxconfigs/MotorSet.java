package frc.robot.sparkmaxconfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


public class MotorSet {
    private final SparkMax lead;


    public MotorSet(SparkMax lead, SparkMax follower, SparkBaseConfig baseConfig, boolean inverted) {
        baseConfig.inverted( inverted );

        lead.configure(
                baseConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig
                .apply(baseConfig)
                .follow(lead)
                .inverted( !inverted );

        follower.configure(
                followerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        this.lead = lead;
    }


    public RelativeEncoder getLeadEncoder() {
        return this.lead.getEncoder();
    }


    public void accept(double value) {
        this.lead.set(value);
    }


    public void stop(){
        this.lead.stopMotor();
    }
}
