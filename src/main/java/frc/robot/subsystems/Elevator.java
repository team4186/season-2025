package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.ElevatorMotorSet;
import java.util.InputMismatchException;

public class Elevator extends SubsystemBase{

    // Motor, Encoder, and Limit Switches variables
    private final ElevatorMotorSet elevatorMotors;
    private final RelativeEncoder relativeEncoder;

    // TODO: CanIds to be installed and implementation
     private final Encoder encoder;
     private final DigitalInput bottomLimitSwitch;
     private final DigitalInput topLimitSwitch;

    private final ProfiledPIDController pid;
    private final ElevatorFeedforward elevatorFeedforward;

    private final SysIdRoutine routine;


    public Elevator(
             DigitalInput bottomLimitSwitch,
             DigitalInput topLimitSwitch,
             ElevatorMotorSet elevatorMotors,
             Encoder encoder,
             ProfiledPIDController pid,
             ElevatorFeedforward elevatorFeedforward
    ) {

        this.elevatorMotors = elevatorMotors;

        // TODO: Replace relative encoder when THRU-BORE Encoder installed
        this.relativeEncoder = this.elevatorMotors.getRelativeEncoder();
        this.relativeEncoder.setPosition(0.0);

        // control
        this.pid = pid;
        this.pid.reset(0.0);
        this.elevatorFeedforward = elevatorFeedforward;

        this.getSubsystem();

        // TODO: Placeholder for later CAN Installation
        // Limit Switches
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;

        // Encoder
        this.encoder = encoder;
        this.encoder.reset();
        // this.encoder.setDistancePerPulse( 0 );

        // SysId Routine for dialing in values for our system
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        this.elevatorMotors::setLeadVoltage,
                        this::logMotors,
                        this));
    }

    // callback reads sensors so that the routine can log the voltage, position, and velocity at each timestep
    private void logMotors(SysIdRoutineLog sysIdRoutineLog) {
//        sysIdRoutineLog.motor("Elevator")
//                .voltage()
//                .linearVelocity()
//                .linearAcceleration();
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }


    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }


    @Override
    public void periodic(){
        // publish smart dashboard info here
        SmartDashboard.putNumber("Elevator_RelativeEncoderDistance", relativeEncoder.getPosition());
        SmartDashboard.putNumber("Elevator_TranslatedDistance", getPositionMeters());
        SmartDashboard.putNumber("Elevator_Velocity", getVelocityMetersPerSecond());

        SmartDashboard.putBoolean("elevator bottom LimitSwitch", UnitsUtility.isBeamBroken(bottomLimitSwitch,false,"elevator bottom switch"));
        SmartDashboard.putBoolean("elevator top LimitSwitch", UnitsUtility.isBeamBroken(topLimitSwitch,false,"elevator top switch"));

        /* TODO: Implement when CANIds established
        SmartDashboard.putNumber("Elevator_EncoderDistance", encoder.getDistance());
        SmartDashboard.putBoolean("Elevator_TopLimitSwitch", topLimitSwitch.get());
        SmartDashboard.putBoolean("Elevator_BottomLimitSwitch", bottomLimitSwitch.get());
        */
    }


    /**
     * This is the thing we can do to find the distance the motor has traveled.
     * To get the distance traveled from a bore encoder, you need to count the number of pulses generated by the encoder
     *  and multiply that number by the "distance per pulse" which is calculated based on the circumference of the bore
     *  and the encoder's resolution (pulses per revolution) - essentially, converting the rotational movement of the
     *  bore into a linear distance traveled.
     * <p>
     * Key steps:
     *      Measure the bore circumference: This is the distance traveled for one full rotation of the bore.
     *      Find the encoder resolution: This is the number of pulses the encoder generates per revolution.
     *      Calculate "distance per pulse": Divide the bore circumference by the encoder resolution.
     *      Read encoder pulses: In your control system, read the number of pulses generated by the encoder.
     *      Calculate distance traveled: Multiply the "distance per pulse" by the number of encoder pulses rea
     */
    public void goToLevel( int requestedLevel ) {
        double levelHeight = getLevelConstant(requestedLevel);
        double topLevel = getLevelConstant(5);
        double bottomLevel = getLevelConstant(0);

        double currentPos = getPositionMeters();
        boolean isPositive =  ( levelHeight - currentPos >= 0 );

        // stop motors if ( negative difference -> check bottom limit OR positive difference -> check top limit )


        //if ( (!isPositive && bottomLevel <= currentPos ) || (isPositive && topLevel >= currentPos )) {
        if ( (!isPositive && UnitsUtility.isBeamBroken(bottomLimitSwitch, true, this.getName())) || (isPositive && UnitsUtility.isBeamBroken(topLimitSwitch, true, this.getName()))) {
            elevatorMotors.stop();
        } else {
            double voltsOutput = MathUtil.clamp(
                    elevatorFeedforward.calculateWithVelocities(
                            getVelocityMetersPerSecond(),
                            pid.getSetpoint().velocity) + pid.calculate(getPositionMeters(), levelHeight), -7, 7);
            elevatorMotors.setLeadVoltage(voltsOutput);
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


    public boolean isAtLevelThreshold( int level ){
        return MathUtil.isNear(
                getPositionMeters(),
                getLevelConstant( level ),
                Constants.ElevatorConstants.ELEVATOR_DEFAULT_TOLERANCE);
    }


    public boolean isAtHeight( double height ) {
        return MathUtil.isNear(
                getPositionMeters(),
                height,
                Constants.ElevatorConstants.ELEVATOR_DEFAULT_TOLERANCE);
    }


    // TODO: BRAINSTORM: Useful for adjusting past breakpoint? should just reset instead probably?
    public void reset() {
        goToLevel(0);
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
    public boolean isAtTop() {
        return topLimitSwitch.get();
    }

    public boolean isAtBottom() {
        return bottomLimitSwitch.get();
    }

    public void stopMotor() {
        elevatorMotors.stop();
    }
}
