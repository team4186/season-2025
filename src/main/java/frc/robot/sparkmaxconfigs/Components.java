package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;


// Components Singleton
public final class Components {

  private static Components instance = null;


  public final ElevatorMotorSet elevatorMotorSet = new ElevatorMotors().elevatorMotors;

  public final SingleMotor endEffectorSingleMotor = new EndEffectorMotor().endEffectorMotor;

  private final AlgaeProcessorMotors algaeProcessorMotors = new AlgaeProcessorMotors();
  public final SingleMotor algaeProcessorWheelSingleMotor =  algaeProcessorMotors.wheelMotor;
  public final SingleMotor algaeProcessorAngleSingleMotor = algaeProcessorMotors.angleMotor;

  private final ClimberMotor climberMotors = new ClimberMotor();
  public final SingleMotor climberSingleMotor =  climberMotors.climbMotor;

  private final DeAlgaeMotors deAlgaeMotors = new DeAlgaeMotors();
  public final SingleMotor deAlgaeWheelSingleMotor = deAlgaeMotors.wheelMotor;
  public final SingleMotor deAlgaeAngleSingleMotor = deAlgaeMotors.angleMotor;


  // private Constructor
  private Components() {}


  public static Components getInstance() {
    if (instance == null) {
      instance = new Components();
    }
    return instance;
  }


  private static final class AlgaeProcessorMotors {
    private final SingleMotor wheelMotor = new SingleMotor(
        new SparkMax(13, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        false);

    private final SingleMotor angleMotor = new SingleMotor(
        new SparkMax(14, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        false);
  }


  private static final class ClimberMotor {
    private final SingleMotor climbMotor = new SingleMotor(
        new SparkMax(15, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        false);
  }


  private static final class DeAlgaeMotors {
    private final SingleMotor wheelMotor = new SingleMotor(
        new SparkMax(10, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        false);

    private final SingleMotor angleMotor = new SingleMotor(
        new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        true);
  }


  private static final class EndEffectorMotor {
    private final SingleMotor endEffectorMotor = new SingleMotor(
        new SparkMax(12, SparkLowLevel.MotorType.kBrushless),
        DefaultMotorConfigs.getInstance().DefaultConfig,
        true);
  }


  private static final class ElevatorMotors {
    private final ElevatorMotorSet elevatorMotors = new ElevatorMotorSet(
        new SparkMax(16, SparkLowLevel.MotorType.kBrushless), // lead
        new SparkMax(17, SparkLowLevel.MotorType.kBrushless), // follower
        DefaultMotorConfigs.getInstance().ElevatorBaseConfig,
        false);
  }
}
