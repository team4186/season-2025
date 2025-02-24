
package frc.robot.sparkmaxconfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


public class YagslElevatorMotorSet {
    public final SparkMax lead;


    public YagslElevatorMotorSet(SparkMax lead, SparkMax follower, SparkBaseConfig baseConfig, boolean inverted) {
        baseConfig.inverted(inverted);

        lead.configure(
                baseConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig
                .apply(baseConfig)
                .inverted(!inverted)
                .follow(lead);

        follower.configure(
                followerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        this.lead = lead;
    }


    public RelativeEncoder getRelativeEncoder() {
        return this.lead.getEncoder();
    }


    public void accept(double value) {
        this.lead.set(value);
    }


    public void stop(){
        this.lead.stopMotor();
    }
}
