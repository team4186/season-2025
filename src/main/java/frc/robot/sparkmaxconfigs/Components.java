package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

// Components Singleton
public final class Components {

    private static Components instance = null;

    public SingleMotor algaeProcessorMotor = new AlgaeProcessorMotor().algaeWheelMotor;
    public SingleMotor algaeProcessorAngleMotor = new AlgaeProcessorMotor().algaeProcessorAngleMotor;
    public MotorSet elevatorMotors = new ElevatorMotors().elevatorMotors;

    public YagslElevatorMotorSet elevatorMotorsYagsl = new ElevatorMotorsYagsl().elevatorMotorsYagsl;
    public SingleMotor endEffectorMotor = new EndEffectorMotor().endEffectorMotor;
    public SingleMotor climberMotor = new ClimberMotor().climberMotor;

    public SingleMotor deAlgaeWheelMotor = new DeAlgaeMotors().wheelMotor;
    public SingleMotor deAlgaeAngleMotor = new DeAlgaeMotors().angleMotor;


    // private Constructor
    private Components() {}


    public static Components getInstance(){
        if (instance == null){
            instance = new Components();
        }
        return instance;
    }


    // TODO: Set proper CAN ID.
    public static final class AlgaeProcessorMotor {
        public SingleMotor algaeWheelMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);

        public SingleMotor algaeProcessorAngleMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().HoldingConfig);
    }


    public static final class ClimberMotor {
        public SingleMotor climberMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);
    }


    public static final class DeAlgaeMotors {
        public SingleMotor wheelMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);

        public SingleMotor angleMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);
    }


    //TODO: Set proper CAN ID.
    public static final class ElevatorMotors {
        public final MotorSet elevatorMotors = new MotorSet(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // lead
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless), // follower
                DefaultMotorConfigs.getInstance().SparkElevatorConfig,
                false
        );
    }


    public static final class ElevatorMotorsYagsl {
        public final YagslElevatorMotorSet elevatorMotorsYagsl = new YagslElevatorMotorSet(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // lead
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless), // follower
                DefaultMotorConfigs.getInstance().SparkElevatorConfig
        );
    }


    // TODO: change IDs
    public static final class EndEffectorMotor {
        public final SingleMotor endEffectorMotor = new SingleMotor(
                new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig);
    }
}
