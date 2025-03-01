package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.SingleMotor;


public class EndEffector extends SubsystemBase {
    private final SingleMotor endEffectorMotor;
    private final DigitalInput luna;


    public EndEffector(SingleMotor endEffectorMotor, DigitalInput luna){
        this.luna = luna;
        this.endEffectorMotor = endEffectorMotor;
    }


    // TODO: Update logic when luna is installed
    public boolean hasGamePiece(){
        return false;
        // return luna.get();
    }


    public void intake(){
        try {
            if ( hasGamePiece() ) {
                endEffectorMotor.stop();
                return;
            }

            endEffectorMotor.accept( Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_SPEED );

        } catch (IllegalStateException e) {
            endEffectorMotor.stop();
            String msg = "EndEffector Beambreak error: " + e;
            System.out.println(msg);
        }
    }


    public void eject() {
        endEffectorMotor.accept( Constants.EndEffectorConstants.END_EFFECTOR_EJECT_SPEED );
    }


    public void stop(){
        endEffectorMotor.stop();
    }

    public void updateSmartDashboard() {
    }
}
