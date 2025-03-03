package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeProcessor;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Elevator;

// Components Singleton
public final class Components {

    private static Components instance = null;

    public final ElevatorMotorSet elevatorMotors = new ElevatorMotors().elevatorMotors;

    public final SingleMotor endEffectorMotor = new EndEffectorMotor().endEffectorMotor;

    public final SingleMotor climberMotor = new ClimberMotor().climberMotor;

    private final AlgaeProcessorMotors algaeProcessorMotors = new AlgaeProcessorMotors();
    public final SingleMotor algaeProcessorWheelMotor =  algaeProcessorMotors.wheelMotor;
    public final SingleMotor algaeProcessorAngleMotor = algaeProcessorMotors.angleMotor;

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


    public static final class AlgaeProcessorMotors {
        public SingleMotor wheelMotor = new SingleMotor(
                new SparkMax(98, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);

        public SingleMotor angleMotor = new SingleMotor(
                new SparkMax(99, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                true);
    }


    public static final class ClimberMotor {
        public SingleMotor climberMotor = new SingleMotor(
                new SparkMax(15, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);
    }


    public static final class DeAlgaeMotors {
        public SingleMotor wheelMotor = new SingleMotor(
                new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                false);

        public SingleMotor angleMotor = new SingleMotor(
                new SparkMax(10, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                true);
    }


    public static final class EndEffectorMotor {
        public final SingleMotor endEffectorMotor = new SingleMotor(
                new SparkMax(12, SparkLowLevel.MotorType.kBrushless),
                DefaultMotorConfigs.getInstance().DefaultConfig,
                true);
    }


    public static final class ElevatorMotors {
        public final ElevatorMotorSet elevatorMotors = new ElevatorMotorSet(
                new SparkMax(16, SparkLowLevel.MotorType.kBrushless), // lead
                new SparkMax(17, SparkLowLevel.MotorType.kBrushless), // follower
                DefaultMotorConfigs.getInstance().ElevatorBaseConfig,
                false);
    }
}
