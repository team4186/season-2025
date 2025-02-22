package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

// MotorConfigs Singleton
public class DefaultMotorConfigs {
    private static DefaultMotorConfigs instance = null;

    public final SparkMaxConfig DefaultConfig = (SparkMaxConfig) new SparkMaxConfig()
            .smartCurrentLimit(50)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);

    public final SparkMaxConfig HoldingConfig = (SparkMaxConfig) new SparkMaxConfig()
            .smartCurrentLimit(10)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);

    public final SparkBaseConfig DefaultLeftMotorConfig = new SparkMaxConfig()
            .apply(DefaultConfig)
            .inverted(true);

    public final SparkBaseConfig DefaultRightMotorConfig = new SparkMaxConfig()
            .apply(DefaultConfig)
            .inverted(false);

    public final SparkBaseConfig SparkElevatorConfig = new SparkMaxConfig()
            .smartCurrentLimit(40)
            .openLoopRampRate(Constants.ElevatorYAGSLConstants.ElevatorRampRate);
            
    private DefaultMotorConfigs(){ }


    public static DefaultMotorConfigs getInstance(){
        if (instance == null){
            instance = new DefaultMotorConfigs();
        }
        return instance;
    }
}

