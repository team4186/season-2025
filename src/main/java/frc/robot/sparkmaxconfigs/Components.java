package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Elevator;

// Components Singleton
public final class Components {

    private static Components instance = null;
    //TODO: Uncomment Later
//    private final AlgaeProcessorMotor algaeProcessorMotorControllers = new AlgaeProcessorMotor();
//    public final SingleMotor algaeProcessorMotor = algaeProcessorMotorControllers.algaeWheelMotor;
//    public final SingleMotor algaeProcessorAngleMotor = algaeProcessorMotorControllers.algaeProcessorAngleMotor;
//
//    public final ElevatorMotorSet elevatorMotors = new ElevatorMotors().elevatorMotors;
    // public YagslElevatorMotorSet elevatorMotorsYagsl = new ElevatorMotorsYagsl().elevatorMotorsYagsl;

    public final SingleMotor endEffectorMotor = new EndEffectorMotor().endEffectorMotor;
    //TODO:Uncomment LAater
//    public final SingleMotor climberMotor = new ClimberMotor().climberMotor;

    private final DeAlgaeMotors deAlgaeMotors = new DeAlgaeMotors();
    public final SingleMotor deAlgaeWheelMotor = deAlgaeMotors.wheelMotor;
    public final SingleMotor deAlgaeAngleMotor = deAlgaeMotors.angleMotor;


    // private Constructor
    private Components() { }


    public static Components getInstance(){
        if (instance == null){
            instance = new Components();
        }
        return instance;
    }

    //TODO: Uncomment, fix ID's, add to chain
//    public static final class AlgaeProcessorMotor {
//        public SingleMotor algaeWheelMotor = new SingleMotor(
//                new SparkMax(13, SparkLowLevel.MotorType.kBrushless),
//                DefaultMotorConfigs.getInstance().DefaultConfig,
//                false);
//
//        public SingleMotor algaeProcessorAngleMotor = new SingleMotor(
//                new SparkMax(14, SparkLowLevel.MotorType.kBrushless),
//                DefaultMotorConfigs.getInstance().HoldingBaseConfig,
//                false);
//    }

    //TODO: Uncomment, get ID's
//    public static final class ClimberMotor {
//        public SingleMotor climberMotor = new SingleMotor(
//                new SparkMax(15, SparkLowLevel.MotorType.kBrushless),
//                DefaultMotorConfigs.getInstance().DefaultConfig,
//                false);
//    }


    public static final class DeAlgaeMotors {
        public SingleMotor wheelMotor = new SingleMotor(
                new SparkMax(10, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);

        public SingleMotor angleMotor = new SingleMotor(
                new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);
    }


    //TODO: Set proper CAN ID.
    //TODO: Uncomment, get ID's
//    public static final class ElevatorMotors {
//        public final ElevatorMotorSet elevatorMotors = new ElevatorMotorSet(
//                new SparkMax(16, SparkLowLevel.MotorType.kBrushless), // lead
//                new SparkMax(17, SparkLowLevel.MotorType.kBrushless), // follower
//                DefaultMotorConfigs.getInstance().ElevatorBaseConfig,
//                false);
//    }
//
//
//    public static final class ElevatorMotorsYagsl {
//        public final YagslElevatorMotorSet elevatorMotorsYagsl = new YagslElevatorMotorSet(
//                new SparkMax(16, SparkLowLevel.MotorType.kBrushless), // lead
//                new SparkMax(17, SparkLowLevel.MotorType.kBrushless), // follower
//                DefaultMotorConfigs.getInstance().ElevatorBaseConfig,
//                false);
//    }


    public static final class EndEffectorMotor {
        public final SingleMotor endEffectorMotor = new SingleMotor(
                new SparkMax(12, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);
    }
}
