package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

// MotorConfigs Singleton for Subsystem Motors (Swerve Subsystem not included)
public final class DefaultMotorConfigs {
    private static DefaultMotorConfigs instance = null;


    public final SparkMaxConfig DefaultConfig = (SparkMaxConfig) new SparkMaxConfig()
            .smartCurrentLimit(50)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);


    public final SparkMaxConfig HoldingBaseConfig = (SparkMaxConfig) new SparkMaxConfig()
            .apply(DefaultConfig)
            .smartCurrentLimit(10);


    public final SparkBaseConfig ElevatorBaseConfig = new SparkMaxConfig()
            .apply(DefaultConfig)
            .smartCurrentLimit(40)
            .openLoopRampRate(Constants.ElevatorConstants.ELEVATOR_RAMP_RATE)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);



    private DefaultMotorConfigs(){ }


    public static DefaultMotorConfigs getInstance(){
        if (instance == null){
            instance = new DefaultMotorConfigs();
        }
        return instance;
    }
}

