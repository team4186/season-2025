package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

//TODO: check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private final SingleMotor motor;
    private final RelativeEncoder encoder;

    private final PIDController PIDController;
    private final double targetAngle;

     private final DigitalInput beamBreak;

    public Climber(
            SingleMotor motor,
            DigitalInput beamBreak,
            PIDController PIDController,
            double targetAngle){
        this.motor = motor;
        this.encoder = motor.getRelativeEncoder();
        this.beamBreak = beamBreak;
        this.PIDController = PIDController;
        this.targetAngle = targetAngle;
    }


    @Override
    public void periodic(){
        /* TODO:
        SmartDashboard.putNumber("DeAlgae Angle:", getCurrentAngle());
        SmartDashboard.putNumber("DeAlgae Speed:", getCurrent_Speed());
        */
    }


    //Avoid using for now, no safeties
    public void deployClimb() {
        if (getEncoderPos() < targetAngle){
            motor.setVoltage(MathUtil.clamp(PIDController.calculate(getEncoderPos(), targetAngle),
                    Constants.ClimberConstants.MINSPEED,
                    Constants.ClimberConstants.MAXSPEED));
            encoder.setPosition(0.0);
        }
    }


    public void stowClimb(){
        if (!UnitsUtility.isBeamBroken(beamBreak, false, this.getName()) ) {
            motor.setVoltage(MathUtil.clamp(PIDController.calculate(getEncoderPos(), -targetAngle),
                    Constants.ClimberConstants.MINSPEED, Constants.ClimberConstants.MAXSPEED));
            encoder.setPosition(0.0);
        } else {
            motor.stop();
        }
    }


    //engageClimb uses beam breaks, has safety and uses higher voltage than deploy
    public void engageClimb() {
        if ( UnitsUtility.isBeamBroken(beamBreak, true, "ClimberSubsystem")) {
            motor.stop();
        } else {
            motor.setVoltage(Constants.ClimberConstants.CLIMBER_CLIMB_VOLTAGE);//only use if motor requires power while up
        }
    }


    // processorPos is the current position of the processor encoder ticks.
    public void stop() {
        motor.stop();
    }


    public void resetEncoder() {
        encoder.setPosition(0.0);
    }


    public double getEncoderPos() {
        return UnitsUtility.ticksToDegrees(encoder.getPosition(), Constants.ClimberConstants.GEARRATIO);
    }
}
