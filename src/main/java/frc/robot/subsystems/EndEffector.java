package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.SingleMotor;


public class EndEffector extends SubsystemBase {
    private final SingleMotor endEffectorMotor;
    private final RelativeEncoder relativeEncoder;
    private final DigitalInput luna;


    public EndEffector(SingleMotor endEffectorMotor, DigitalInput luna){
        this.luna = luna;
        this.endEffectorMotor = endEffectorMotor;
        this.relativeEncoder = endEffectorMotor.getRelativeEncoder();
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        SmartDashboard.putBoolean("hasGamePiece", hasGamePiece());
        SmartDashboard.putNumber("EndEffectorVoltage", relativeEncoder.getVelocity());
    }


    // TODO: Update logic when luna is installed
    public boolean hasGamePiece(){

        try {
            if ( luna.get() ) {
                return true;
            }
        } catch (IllegalStateException e) {
            String msg = "EndEffector Beambreak error: " + e;
            System.out.println(msg);
        }

        // return false if not confident about item
        return false;
    }


    public void intake(){
        if ( hasGamePiece() ) {
            endEffectorMotor.stop();
        } else {
            endEffectorMotor.setVoltage( Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE );
        }
    }


    public void eject() {
        if ( hasGamePiece() ) {
            endEffectorMotor.setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_EJECT_VOLTAGE);
        } else {
            endEffectorMotor.stop();
        }
    }


    public void stop(){
        endEffectorMotor.stop();
    }
}
