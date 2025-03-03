package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.SingleMotor;


public class EndEffector extends SubsystemBase {
    private final SingleMotor endEffectorMotor;
    private final RelativeEncoder relativeEncoder;
    private final DigitalInput tfLuna;


    public EndEffector(SingleMotor endEffectorMotor, DigitalInput tfLuna){
        this.tfLuna = tfLuna;
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
        return UnitsUtility.isBeamBroken( tfLuna, true, this.getName());
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


    public void testSpeeds(double voltage){
        endEffectorMotor.setVoltage(voltage);
    }
}
