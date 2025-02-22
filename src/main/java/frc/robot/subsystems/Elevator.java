package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.MotorSet;

public class Elevator {

    // Motor, Encoder, and Limit Switches variables
    private final MotorSet elevatorMotors = Components.getInstance().elevatorMotors;
    private final RelativeEncoder encoder;

    // Make id # correct
    private final DigitalInput bottomLimitSwitch = new DigitalInput(2);
    private final DigitalInput topLimitSwitch = new DigitalInput(3);

    private final PIDController pid = new PIDController(
    Constants.ElevatorConstants.PROPORTIONAL,
    Constants.ElevatorConstants.INTEGRAL,
    Constants.ElevatorConstants.DERIVATIVE);

    private int level;
    private double currentElevatorHeight;

    public Elevator() {
//        encoder = elevatorMotors.getLeadEncoder();
//        encoder.setPosition(0.0);
        this.encoder = elevatorMotors.getLeadEncoder();
        this.currentElevatorHeight = this.encoder.getPosition();
    }

    /*
        This is the thing we can do to find the distance the motor has traveled.

        To get the distance traveled from a bore encoder, you need to count the number of pulses generated by the encoder and multiply that number by the "distance per pulse" which is calculated based on the circumference of the bore and the encoder's resolution (pulses per revolution) - essentially, converting the rotational movement of the bore into a linear distance traveled.
Key steps:
Measure the bore circumference: This is the distance traveled for one full rotation of the bore.
Find the encoder resolution: This is the number of pulses the encoder generates per revolution.
Calculate "distance per pulse": Divide the bore circumference by the encoder resolution.
Read encoder pulses: In your control system, read the number of pulses generated by the encoder.
Calculate distance traveled: Multiply the "distance per pulse" by the number of encoder pulses rea
         */

    public void goToLevel(int controllerInput) {
        double distanceToLevel;

        // TODO: Pass requested level to move function
        switch (controllerInput) {
            case 1:
                distanceToLevel = Constants.ElevatorConstants.LEVEL_ONE_HEIGHT - getEncoderDistance();
                goUp(distanceToLevel, Constants.ElevatorConstants.LEVEL_ONE_HEIGHT);
                break;
            case 2:
                distanceToLevel = Constants.ElevatorConstants.LEVEL_TWO_HEIGHT - getEncoderDistance();
                goUp(distanceToLevel, Constants.ElevatorConstants.LEVEL_TWO_HEIGHT);
                break;
            case 3:
                distanceToLevel = Constants.ElevatorConstants.LEVEL_THREE_HEIGHT - getEncoderDistance();
                goUp(distanceToLevel, Constants.ElevatorConstants.LEVEL_THREE_HEIGHT);
                break;
        }
    }

    public void goUp(double distanceToLevel, double height) {
        double speed = coerceIn(pid.calculate(getEncoderDistance(), height),
                -Constants.ElevatorConstants.DEFAULT_FREE_MOVE_SPEED,
                Constants.ElevatorConstants.DEFAULT_FREE_MOVE_SPEED);

        // TODO: Need to include tolerance for double comparison!
        if (distanceToLevel == 0) {
            setEncoderDistance(height);
            stopMotor();
        } else {
            elevatorMotors.setSpeed(speed);
        }
    }

    public void goDown() {
        double speed = coerceIn(pid.calculate(getEncoderDistance(), 0.0),
                -Constants.ElevatorConstants.DEFAULT_FREE_MOVE_SPEED,
                Constants.ElevatorConstants.DEFAULT_FREE_MOVE_SPEED);

        if (!bottomLimitSwitch.get()) {  //might have to change if bottomLimitSwitch is false when activated
            elevatorMotors.setSpeed(speed);
        } else {
            encoder.setPosition(0.0);
            stopMotor();
        }
    }

    public double getEncoderDistance() {
        return encoder.getPosition() * Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR;
    }

    public void setEncoderDistance(double height) {
        encoder.setPosition(height / Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR);
    }

    public double coerceIn(double value, double lowerBound, double upperBound) {
        if (value > upperBound) {
            return upperBound;
        } else if (value < lowerBound) {
            return lowerBound;
        } else {
            return value;
        }
    }

    public void stopMotor() {
        elevatorMotors.stop();
        pid.reset();
    }
}