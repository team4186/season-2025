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


    public static class ClimberMotor {
        // TODO
    }


    public static class DeAlgaeMotor {
        // TODO
    }


    public static class ElevatorMotors {
        public final MotorSet elevatorMotors = new MotorSet(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // lead
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless), // follower
                DefaultMotorConfigs.getInstance().DefaultConfig
        );
    }


    // TODO: change IDs
    public static final class EndEffectorMotor {
        public final SingleMotor endEffectorMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);
    }
}
