package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;

//TODO: check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private final SingleMotor motor = Components.getInstance().climberMotor;
    private final RelativeEncoder encoder  = motor.getEncoder();
    // Change the digital input channel later.
    private final DigitalInput beamBreak  = new DigitalInput(0);

    //Avoid using for now, no safeties
    public void deployClimb(){
        motor.setVoltage(Constants.ClimberConstants.CLIMBER_MOVE_VOLTAGE);
    }

    public void stowClimb(){
        if (!beamBreak.get()) {
            motor.setVoltage(-Constants.ClimberConstants.CLIMBER_MOVE_VOLTAGE);
        }
    }

    //engageClimb uses beam breaks, has safety and uses higher voltage than deploy
    public void engageClimb() {
        try {
            if (beamBreak.get()) {
                motor.stop();
            } else {
                motor.setVoltage(Constants.ClimberConstants.CLIMBER_CLIMB_VOLTAGE);//only use if motor requires power while up
            }
        } catch (IllegalStateException e) {
            motor.stop();
            String msg = "Climber Beambreak error: " + e.toString();
            System.out.println( msg );
        }
    }

    public void stop() {
        motor.stop();
    }
}
