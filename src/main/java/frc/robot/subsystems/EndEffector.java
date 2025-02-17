package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;

public class EndEffector extends SubsystemBase {
    private final SingleMotor endEffectorMotor = Components.getInstance().endEffectorMotor;
    //private RelativeEncoder encoder;
    private DigitalInput luna;

    public EndEffector(){
        //encoder = endEffectorMotor.motor.getEncoder();
        luna = new DigitalInput(Constants.EndEffectorConstants.beamBreakChannel);
    }

    public Command intake(){
        try {
            if (!luna.get()) {
                endEffectorMotor.setSpeed(Constants.EndEffectorConstants.speed);
            } else {
                endEffectorMotor.stop();
            }

        } catch (IllegalStateException e) {
            endEffectorMotor.stop();
            String msg = "EndEffector Beambreak error: " + e.toString();
            System.out.println(msg);
        }

        return null;
    }

    public Command eject() {
        endEffectorMotor.setSpeed(-Constants.EndEffectorConstants.speed);

        return null;
    }

    public Command stop(){
        endEffectorMotor.stop();

        return null;
    }
}
