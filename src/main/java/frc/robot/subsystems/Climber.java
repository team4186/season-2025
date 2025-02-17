package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: Update sparkID, sensorID, climbVoltage, moveVoltage,
// check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private static final int sparkID = 0;
    //Climb voltage requires much higher values than move voltage.
    private static final int climbVoltage = 0;
    private static final int moveVoltage = climbVoltage / 3;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput beamBreak;


    public Climber( int beamBreakId ){
        beamBreak = new DigitalInput(beamBreakId);
        motor = new SparkMax(sparkID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    //Avoid using for now, no safeties
    public Command deployClimb(){
        motor.setVoltage(moveVoltage);
        return null;
    }


    public Command stowClimb(){
        if (!beamBreak.get()) {
            motor.setVoltage(-moveVoltage);
        }
        return null;
    }

    //engageClimb uses beam breaks, has safety and uses higher voltage than deploy
    public Command engageClimb() {
        try {
            if (beamBreak.get()) {
                motor.stopMotor();
            } else {
                motor.setVoltage(climbVoltage);//only use if motor requires power while up
            }
        } catch (IllegalStateException e) {
            motor.stopMotor();
            String msg = "Climber Beambreak error: " + e.toString();
            System.out.println( msg );
        }
        return null;
    }
}
