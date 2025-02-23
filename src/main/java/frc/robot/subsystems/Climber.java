package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Units;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;

//TODO: check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private final SingleMotor motor;
    private final RelativeEncoder encoder;
    // Change the digital input channel later.
    private final DigitalInput beamBreak;
    private final PIDController DIPController;
    private final double targetAngle;
    private final double MAXVOLTS;
    private final double MINVOLTS;

    public Climber(SingleMotor motor, DigitalInput beamBreak, PIDController DIPController, double targetAngle, double MAXVOLTS, double MINVOLTS) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.beamBreak = beamBreak;
        this.DIPController = DIPController;
        this.targetAngle = targetAngle;
        this.MAXVOLTS = MAXVOLTS;
        this.MINVOLTS = MINVOLTS;
    }
    //Avoid using for now, no safeties
    public void deployClimb() {
        if (getEncoderPos() < targetAngle){
            motor.setVoltage(coerceIn(DIPController.calculate(getEncoderPos(), targetAngle)));
            encoder.setPosition(0.0);
        }
    }

    public void stowClimb(){
        if (!beamBreak.get()) {
            motor.setVoltage(coerceIn(DIPController.calculate(getEncoderPos(), -targetAngle)));
            encoder.setPosition(0.0);
        } else {
            motor.stop();
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

    public void resetEncoder() {
        encoder.setPosition(0.0);
    }

    public double getEncoderPos() {
        return Units.TicksToDegrees(encoder.getPosition(), Constants.ClimberConstants.GEARRATIO);
    }

    public double coerceIn(double value) {
        // Fail safe
        double output = 0.0;
        if (Math.abs(value) >= MAXVOLTS) {
            output = Math.copySign(MAXVOLTS, value);
        } else if (Math.abs(value) <= MINVOLTS) {
            output = Math.copySign(MINVOLTS, value);
        }
        return output;
    }
}
