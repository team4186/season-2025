package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// Components Singleton
public final class Components {
    private static Components instance = null;
    public SingleMotor algaeProcessorMotor = new AlgaeProcessorMotor().algaeMotor;
    public SingleMotor processorDeployMotor = new AlgaeProcessorMotor().deployMotor;
    // private Constructor
    private Components() {}


    public static Components getInstance(){
        if (instance == null){
            instance = new Components();
        }
        return instance;
    }


    // TODO: Set default configuration
    public static final class AlgaeProcessorMotor {
        // private final MotorConfigs motorConfigs = MotorConfigs.getInstance();
        public final SparkMax motor;
        public final SparkMaxConfig config;
        public final SparkMax motor2;
        public final SparkMaxConfig config2;

        public SingleMotor algaeMotor = new SingleMotor(
                motor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                config = DefaultMotorConfigs.getInstance().DefaultConfig);
        public SingleMotor deployMotor = new SingleMotor(
                motor2 = new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
                config2 = DefaultMotorConfigs.getInstance().HoldingConfig);
    }

    public static class ClimberMotor { }
    public static class DeAlgaeMotor { }
    public static class ElevatorMotors { }
    public static class EndEffectorMotor { }
}
