package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

// MotorConfigs Singleton
public class DefaultMotorConfigs {
    private static DefaultMotorConfigs instance = null;

    public final SparkMaxConfig DefaultConfig = (SparkMaxConfig) new SparkMaxConfig()
            .smartCurrentLimit(50)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);

    public final SparkBaseConfig DefaultLeftMotorConfig = new SparkMaxConfig()
            .apply(DefaultConfig)
            .inverted(true);

    public final SparkBaseConfig DefaultRightMotorConfig = new SparkMaxConfig()
            .apply(DefaultConfig)
            .inverted(false);


    private DefaultMotorConfigs(){ }


    public static DefaultMotorConfigs getInstance(){
        if (instance == null){
            instance = new DefaultMotorConfigs();
        }
        return instance;
    }
}

