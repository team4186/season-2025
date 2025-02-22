package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.MotorSet;

import java.util.InputMismatchException;

public class Elevator {

    // Motor, Encoder, and Limit Switches variables
    private final MotorSet elevatorMotors;
    private final Encoder encoder;

    // Make id # correct
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private final PIDController pid;


    public Elevator(
            DigitalInput bottomLimitSwitch,
            DigitalInput topLimitSwitch,
            MotorSet elevatorMotor,
            Encoder encoder,
            PIDController pid
    ) {
        this.elevatorMotors = elevatorMotor;
        this.encoder = encoder;
        this.pid = pid;

        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;

        pid.reset();
        encoder.reset();
        encoder.setDistancePerPulse( 0 ); // TODO: Set distance per pulse here
    }

    /*
        This is the thing we can do to find the distance the motor has traveled.

    /**
     * This is the thing we can do to find the distance the motor has traveled.
     *
     * To get the distance traveled from a bore encoder, you need to count the number of pulses generated by the encoder and multiply that number by the "distance per pulse" which is calculated based on the circumference of the bore and the encoder's resolution (pulses per revolution) - essentially, converting the rotational movement of the bore into a linear distance traveled.
     *
     *  Key steps:
     *      Measure the bore circumference: This is the distance traveled for one full rotation of the bore.
     *      Find the encoder resolution: This is the number of pulses the encoder generates per revolution.
     *      Calculate "distance per pulse": Divide the bore circumference by the encoder resolution.
     *      Read encoder pulses: In your control system, read the number of pulses generated by the encoder.
     *      Calculate distance traveled: Multiply the "distance per pulse" by the number of encoder pulses rea
     */
    public void goToLevel(int requestedLevel) {
        double distanceToLevel;
        double levelHeight;

        double currentPos = encoder.getDistance();

        // TODO: Pass requested level to move function
        levelHeight = switch (requestedLevel) {
            case 1 -> {
                distanceToLevel = Constants.ElevatorConstants.ELEVATOR_LEVEL_ONE - currentPos;
                yield Constants.ElevatorConstants.ELEVATOR_LEVEL_ONE;
            }
            case 2 -> {
                distanceToLevel = Constants.ElevatorConstants.ELEVATOR_LEVEL_TWO - currentPos;
                yield Constants.ElevatorConstants.ELEVATOR_LEVEL_TWO;
            }
            case 3 -> {
                distanceToLevel = Constants.ElevatorConstants.ELEVATOR_LEVEL_THREE - currentPos;
                yield Constants.ElevatorConstants.ELEVATOR_LEVEL_THREE;
            }
            case 4 -> {
                distanceToLevel = Constants.ElevatorConstants.ELEVATOR_LEVEL_FOUR - currentPos;
                yield Constants.ElevatorConstants.ELEVATOR_LEVEL_FOUR;
            }
            default -> throw new InputMismatchException("Received unexpected requested elevator checkpoint");
        };

        double speed = coerceIn(pid.calculate(distanceToLevel, levelHeight),
                -Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED,
                Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED);

        moveUp(speed, levelHeight);
    }


    // public void goUp(double distanceToLevel, double height) {
    public void moveUp( double speed, double height) {
        // TODO: Need to include tolerance for double comparison!
        if ( height  >= 0 || topLimitSwitch.get() ) {
            stopMotor();
        } else {
            elevatorMotors.accept(speed);
        }
    }


    // TODO: BRAINSTORM: Useful for adjusting past breakpoint? should just reset instead probably?
    public void reset() {
        double speed = coerceIn(pid.calculate(getEncoderDistance(), 0.0),
                -Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED,
                Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED);

        // TODO: Need to catch not finding true/false result?
        if (bottomLimitSwitch.get()) {  //might have to change if bottomLimitSwitch is false when activated
            encoder.reset();
            stopMotor();
        } else {
            elevatorMotors.accept(speed);
        }
    }


    // TODO: BEFORE TESTING Replace with setting in configs
    public double getEncoderDistance() {
        // return encoder.getPosition() * Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR;
        return 0.0;
    }


    // TODO: BEFORE TESTING Replace with settings in configs
    public void setEncoderDistance(double height) {
        // encoder.setPosition(height / Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR);
    }


    public double coerceIn(double value, double lowerBound, double upperBound) {
        if (value > upperBound) {
            return upperBound;
        } else {
            return Math.max(value, lowerBound);
        }
    }


    public void stopMotor() {
        elevatorMotors.stop();
        pid.reset();
    }
}