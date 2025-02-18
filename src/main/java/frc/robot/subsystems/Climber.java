package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;

//TODO: check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private final SingleMotor motor = Components.getInstance().climberMotor;
    private final RelativeEncoder encoder  = motor.getLeadEncoder();
    // Change the digital input channel later.
    private final DigitalInput beamBreak  = new DigitalInput(0);

    //Avoid using for now, no safeties
    public Command deployClimb(){
        motor.setVoltage(Constants.ClimberConstants.moveVoltage);
        return null;
    }


    public Command stowClimb(){
        if (!beamBreak.get()) {
            motor.setVoltage(-Constants.ClimberConstants.moveVoltage);
        }
        return null;
    }

    //engageClimb uses beam breaks, has safety and uses higher voltage than deploy
    public Command engageClimb() {
        try {
            if (beamBreak.get()) {
                motor.stopMotor();
            } else {
                motor.setVoltage(Constants.ClimberConstants.climbVoltage);//only use if motor requires power while up
            }
        } catch (IllegalStateException e) {
            motor.stopMotor();
            String msg = "Climber Beambreak error: " + e.toString();
            System.out.println( msg );
        }
        return null;
    }
}
