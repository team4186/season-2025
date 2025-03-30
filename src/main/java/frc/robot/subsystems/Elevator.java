package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import frc.robot.sparkmaxconfigs.ElevatorMotorSet;
import java.util.InputMismatchException;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {

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

    private final Timer timer;
    private double prevTimestamp;
    private double prevVelocity;

    private double maxVel;
    private double minVel;
    private double minAccel;
    private double maxAccel;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);


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

        // Limit Switches
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;

        this.encoder = encoder;
        this.encoder.setDistancePerPulse( 1.06938/6626.506 );
        this.encoder.setMinRate(0.0075); // MetersPerSecond

        // SysId Routine for updating FeedForward values
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3.5), null),
                // new SysIdRoutine.Config(), // Default: 1 Volts/Second, 7 Volts
                new SysIdRoutine.Mechanism(
                        this.elevatorMotors::setLeadVoltage,
                        this::logMotors,
                        this));

        this.timer = new Timer();
        timer.start();
        prevTimestamp = 0.0;
        prevVelocity = 0.0;

        maxVel = 0;
        minVel = 0;
        minAccel = 0;
        maxAccel = 0;

    }


    // callback reads sensors so that the routine can log the voltage, position, and velocity at each timestamp
    private void logMotors(SysIdRoutineLog sysIdRoutineLog) {
        sysIdRoutineLog.motor("Elevator")
                .voltage(
                        Volts.of(elevatorMotors.getLead().getBusVoltage() * RobotController.getBatteryVoltage()))
                .linearPosition(
                        Meters.of(encoder.get()))
                .linearVelocity(
                        MetersPerSecond.of(encoder.getRate()));
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
//        SmartDashboard.putNumber("Elevator_RelativeEncoder_Distance", relativeEncoder.getPosition());
//        SmartDashboard.putNumber("Elevator_Velocity_Calculated", getVelocityMetersPerSecond());
        maxVel = Math.max( encoder.getRate(), maxVel);
        minVel = Math.min( encoder.getRate(), minVel);
        maxAccel = Math.max( getAcceleration(), maxAccel);
        minAccel = Math.min( getAcceleration(), minAccel);

        SmartDashboard.putNumber("Elevator_BoreEncoder_Distance", encoder.getDistance());
        SmartDashboard.putNumber("Elevator_BoreEncoder_Rate", encoder.getRate());
        SmartDashboard.putNumber("Elevator_Requested_Velocity", Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY);
//        SmartDashboard.putNumber("Elevator_Velocity_Difference", Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY - encoder.getRate());

        SmartDashboard.putNumber("Elevator_Velocity_Var", maxVel - minVel);
        SmartDashboard.putNumber("Elevator_Velocity_Max", maxVel);
        SmartDashboard.putNumber("Elevator_Acceleration_Var", maxAccel - minAccel);
        SmartDashboard.putNumber("Elevator_Acceleration_Max", maxAccel);

        SmartDashboard.putNumber("Elevator_Acceleration_Expected", Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
//        SmartDashboard.putNumber("Elevator_Acceleration_Difference", Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION - getAcceleration());



//        SmartDashboard.putBoolean("Elevator_LimitSwitch_Top", topLimitSwitch.get());
//        SmartDashboard.putBoolean("Elevator_LimitSwitch_Bottom", bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Elevator_LimitSwitch_Top", getTopBeamBreak());
        SmartDashboard.putBoolean("Elevator_LimitSwitch_Bottom", getBottomBeamBreak());
    }

    private double getAcceleration() {
        double currVelocity = encoder.getRate();
        double currTimestamp = timer.get();
        double prevVelocity = this.prevVelocity;
        double prevTimestamp = this.prevTimestamp;


        double currAccel = (currVelocity - prevVelocity) / (currTimestamp - prevTimestamp);

        this.prevVelocity = currVelocity;
        this.prevTimestamp = currTimestamp;

        return currAccel;
    }


    private boolean getTopBeamBreak(){
        // return true by default to stop motor from exceeding limits
        return UnitsUtility.isBeamBroken(topLimitSwitch,true,"Elevator Top Limit Switch");
    }


    private boolean getBottomBeamBreak(){
        // return true by default to stop motor from exceeding limits
        return !UnitsUtility.isBeamBroken(bottomLimitSwitch,false,"Elevator Bottom Limit Switch");
    }


    /**
     * TODO: measure elevator shaft radius and put it in constants.
     * Ask Chris for the gear ratio for the elevator.
     */
    public void goToLevel( int requestedLevel ) {
        double levelHeight = getLevelConstant(requestedLevel);
        pid.setGoal(levelHeight);

        double adjustment = pid.calculate(encoder.getDistance())
                + elevatorFeedforward.calculate(pid.getSetpoint().velocity);

        //if (velocity is negative AND bottomBeamBreak) OR (velocity is positive AND topBeamBreak) {
        if ((encoder.getRate() < 0 && getBottomBeamBreak()) || (encoder.getRate() > 0 && getTopBeamBreak())) {
            stopMotor();
        } else if ( (encoder.getRate() == 0 && getTopBeamBreak() ) &&  adjustment > 0 ) {
            stopMotor();
        } else {
            elevatorMotors.setLeadVoltage( adjustment );
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


    public boolean isAtLevelThreshold( int level ) {
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
        // Travel to level, slower than normal or just as fast?
        // Move slowly until bottom limit switch?
        goToLevel(0);
    }


    // TODO: BEFORE TESTING Replace with setting in configs
    public double getPositionMeters() {
//        return relativeEncoder.getPosition() *
//                (2 * Math.PI * Constants.ElevatorConstants.ELEVATOR_DRUM_RADIUS)
//                * (1 / Constants.ElevatorConstants.ELEVATOR_GEARING) * 1.179042253521127;
        return encoder.getDistance();
    }


    public double getVelocityMetersPerSecond() {
        return encoder.getRate();
    }


    public boolean isAtTop() {
        return topLimitSwitch.get();
    }


    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();
    }


    public void stopMotor() {

//        elevatorMotors.setLeadVoltage(
//                pid.calculate(encoder.getDistance())
//                        + elevatorFeedforward.calculate(0));
      elevatorMotors.stop();
    }

    public Command slowToStop(){
        return this.run( this::stopMotor );
                //.until(() -> encoder.getRate() >= -0.05 && relativeEncoder.getVelocity() <= 0.05);
    }

    public Runnable applyVoltage(double voltage, boolean isPositive) {
        if (isPositive){
            return () -> elevatorMotors.setLeadVoltage( voltage );
        } else {
            return () -> elevatorMotors.setLeadVoltage( voltage * -1 );
        }
    }
}
