package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Units;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;
import edu.wpi.first.math.MathUtil;

//TODO: check if motor requires voltage to lock, fix return statements
public class Climber extends SubsystemBase {
    private final SingleMotor motor;
    private final RelativeEncoder encoder;
    // Change the digital input channel later.
    private final DigitalInput beamBreak;
    private final PIDController PIDController;
    private final double targetAngle;

    public Climber(SingleMotor motor, DigitalInput beamBreak, PIDController PIDController, double targetAngle) {
        this.motor = motor;
        this.encoder = motor.getRelativeEncoder();
        this.beamBreak = beamBreak;
        this.PIDController = PIDController;
        this.targetAngle = targetAngle;
    }


    @Override
    public void periodic(){
        //SmartDashboard.putNumber("DeAlgae Angle:", getCurrentAngle());
        //SmartDashboard.putNumber("DeAlgae Speed:", getCurrent_Speed());
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
        if (!beamBreak.get()) {
            motor.setVoltage(MathUtil.clamp(PIDController.calculate(getEncoderPos(), -targetAngle),
                    Constants.ClimberConstants.MINSPEED, Constants.ClimberConstants.MAXSPEED));
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


    // processorPos is the current position of the processor encoder ticks.
    public void stop() {
        motor.stop();
    }


    public void resetEncoder() {
        encoder.setPosition(0.0);
    }


    public double getEncoderPos() {
        return Units.TicksToDegrees(encoder.getPosition(), Constants.ClimberConstants.GEARRATIO);
    }

    public void updateSmartDashboard() {
    }
}
