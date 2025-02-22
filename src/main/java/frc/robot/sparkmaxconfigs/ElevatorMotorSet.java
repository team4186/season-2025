
package frc.robot.sparkmaxconfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorMotorSet {
    public final SparkMax lead;
    public ElevatorMotorSet(SparkMax lead, SparkMax follower, SparkBaseConfig baseConfig){
        lead.configure(
                baseConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig
                .apply(baseConfig)
                .follow(lead);

        follower.configure(
                followerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
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
