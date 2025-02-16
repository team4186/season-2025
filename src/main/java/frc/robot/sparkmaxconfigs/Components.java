package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// Components Singleton
public final class Components {
    private static Components instance = null;
    public SingleMotor algaeProcessorMotor = new AlgaeProcessorMotor().algaeMotor;
    public MotorSet elevatorMotors = new ElevatorMotors().elevatorMotors;
    public SingleMotor endEffectorMotor = new EndEffectorMotor().endEffectorMotor;
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
        public SingleMotor algaeMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);
    }

    public static class ClimberMotor { }
    public static class DeAlgaeMotor { }
    public static class ElevatorMotors {
        public MotorSet elevatorMotors = new MotorSet(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // change 0 and 1 for id's
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig
        );
    }

    // TODO: change IDs
    public static final class EndEffectorMotor {
        public final SparkMax motor;
        public final SparkMaxConfig config;

        public SingleMotor endEffectorMotor = new SingleMotor(
                motor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                config = DefaultMotorConfigs.getInstance().DefaultConfig);
    }
}
