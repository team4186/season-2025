package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;


public class Climber {
    private SparkMax motor;
    private DigitalInput sensor;
    private static int port = 15; //place holder
    private static int worker = 12; //place holder
    private static int sped = 1; //place holder


    public Climber(){
        motor = new SparkMax(worker, SparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(port);
    }
    public Command forward (){
        motor.setVoltage(-10);//place holder
    }
    public Command clamp(){
        while (sensor.get()){
          motor.set(sped);
        }
        motor.stopMotor();
    }
}
