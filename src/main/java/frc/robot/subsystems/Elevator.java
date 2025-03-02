package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.ElevatorMotorSet;
import frc.robot.Units;

import java.util.InputMismatchException;

public class Elevator extends SubsystemBase{

    // Motor, Encoder, and Limit Switches variables
    private final ElevatorMotorSet elevatorMotors;
    private final Encoder encoder;
    private final RelativeEncoder relativeEncoder;

    // Make id # correct
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private final ProfiledPIDController pid;
    private final ElevatorFeedforward elevatorFeedforward;


    public Elevator(
            DigitalInput bottomLimitSwitch,
            DigitalInput topLimitSwitch,
            ElevatorMotorSet elevatorMotor,
            Encoder encoder,
            ProfiledPIDController pid,
            ElevatorFeedforward elevatorFeedforward
    ) {
        // motors
        this.elevatorMotors = elevatorMotor;
        this.encoder = encoder;
        this.encoder.reset();

        this.relativeEncoder = this.elevatorMotors.getRelativeEncoder();

        // sensors
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;

        // control
        this.pid = pid;
        this.elevatorFeedforward = elevatorFeedforward;

        // TODO: set elevator to bottom threshold and zero
        //pid.reset(0.0);
        encoder.reset();

        // TODO: assign conversion of elevator distance with rotations of encoder to (SparkMaxConfig or directly to encoder???)
        encoder.setDistancePerPulse( 0 ); // TODO: Set distance per pulse here

        // converte, 1 rotation == distance for relative encoder and encoder
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        SmartDashboard.putNumber("Elevator_EncoderDistance", encoder.getDistance());
        SmartDashboard.putNumber("Elevator_RelativeEncoderDistance", relativeEncoder.getPosition());
        SmartDashboard.putNumber("Elevator_TranslatedDistance", getPositionMeters());
        SmartDashboard.putBoolean("Elevator_TopLimitSwitch", topLimitSwitch.get());
        SmartDashboard.putBoolean("Elevator_BottomLimitSwitch", bottomLimitSwitch.get());
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
    public void goToLevel( int requestedLevel ) {
        double distanceToLevel;
        double levelHeight;

        double currentPos = encoder.getDistance();
        levelHeight = getLevelConstant(requestedLevel);
        distanceToLevel = levelHeight - currentPos;


//        double speed = Units.ClampValue(pid.calculate(distanceToLevel, levelHeight),
//                -Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED,
//                Constants.ElevatorConstants.ELEVATOR_DEFAULT_FREE_MOVE_SPEED);

        double voltsOutput = MathUtil.clamp(
                elevatorFeedforward.calculateWithVelocities(
                        getVelocityMetersPerSecond(),
                        pid.getSetpoint().velocity) + pid.calculate(getPositionMeters(), levelHeight), -7, 7);

        moveToLevel(voltsOutput, levelHeight);
    }


    // public void goUp(double distanceToLevel, double goalHeight) {
    public void moveToLevel( double voltage, double levelHeight) {
        // limit switches
        if ( (voltage >= 0.001 && topLimitSwitch.get()) || (voltage <= 0.001 && bottomLimitSwitch.get()) ) {
            elevatorMotors.stop();
        } else {

        }

    }


    public double getLevelConstant( int level ){
        return switch (level) {
            case 0 -> Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT;
            case 1 -> Constants.ElevatorConstants.ELEVATOR_LEVEL_ONE;
            case 2 -> Constants.ElevatorConstants.ELEVATOR_LEVEL_TWO;
            case 3 -> Constants.ElevatorConstants.ELEVATOR_LEVEL_THREE;
            case 4 -> Constants.ElevatorConstants.ELEVATOR_LEVEL_FOUR;
            case 5 -> Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT;
            default -> throw new InputMismatchException("Received unexpected requested elevator checkpoint");
        };
    }


    // Check
    public boolean isAtLevelThreshold( int level ){
        return ( getPositionMeters() >= getLevelConstant(level) || topLimitSwitch.get() );
    }


    public boolean aroundHeight(double height) {
        return MathUtil.isNear(
                getPositionMeters(),
                height,
                Constants.ElevatorConstants.ELEVATOR_DEFAULT_TOLERANCE);
    }


    // TODO: BRAINSTORM: Useful for adjusting past breakpoint? should just reset instead probably?
    public void reset() {
        goToLevel(0);

        // TODO: Need to catch not finding true/false result?
        if (bottomLimitSwitch.get()) {  //might have to change if bottomLimitSwitch is false when activated
            stopMotor();
            encoder.reset();
            // pid.reset();
        }
    }


    // TODO: BEFORE TESTING Replace with setting in configs
    public double getPositionMeters() {
        return relativeEncoder.getPosition() *
                (2 * Math.PI * Constants.ElevatorConstants.ELEVATOR_DRUM_RADIUS)
                * (1 / Constants.ElevatorConstants.ELEVATOR_GEARING);
    }


    public double getVelocityMetersPerSecond() {
        return (relativeEncoder.getVelocity() / 60) * (2 * Math.PI * Constants.ElevatorConstants.ELEVATOR_DRUM_RADIUS)
                * (1 / Constants.ElevatorConstants.ELEVATOR_GEARING);
    }


    public void stopMotor() {
        elevatorMotors.stop();
        // pid.reset(0.0);
    }
}
