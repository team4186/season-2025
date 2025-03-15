package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
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

        // TODO: Placeholder for later CAN Installation
        // Limit Switches
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;

        // Encoder
        this.encoder = encoder;
        this.encoder.reset();
        this.encoder.setDistancePerPulse( 1.0/2048.0 );

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
        sysIdRoutineLog.motor("Elevator")
                .voltage(
                        m_appliedVoltage.mut_replace(
                                this.elevatorMotors.getLead().get()  * RobotController.getBatteryVoltage(), Volts)
                )
                .linearPosition(
                        m_distance.mut_replace(getPositionMeters(), Meters))
                .linearVelocity(
                        m_velocity.mut_replace(getVelocityMetersPerSecond(), MetersPerSecond));
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

        SmartDashboard.putNumber("Elevator_EncoderDistance", encoder.getDistance());
        SmartDashboard.putNumber("Elevator_EncoderDistancePerPulse", encoder.getDistancePerPulse());

        SmartDashboard.putBoolean("Elevator_TopLimitSwitch", topLimitSwitch.get());
        SmartDashboard.putBoolean("Elevator_BottomLimitSwitch", bottomLimitSwitch.get());

        if ( !SmartDashboard.getBoolean("Elevator_TopLimitSwitch", false) ){
            SmartDashboard.putNumber("Elevator_Encoder_TopLimitSwitchDistance", encoder.getDistance());
        }
    }

    private boolean getTopBeamBreak(){
        return UnitsUtility.isBeamBroken(topLimitSwitch,false,"Elevator bottom switch");
    }

    private boolean getBottomBeamBreak(){
        return UnitsUtility.isBeamBroken(bottomLimitSwitch,false,"DeAlgae limit switch");
    }


    /**
     * TODO: measure elevator shaft radius and put it in constants.
     * Ask Chris for the gear ratio for the elevator.
     */
    public void goToLevel( int requestedLevel ) {
        double levelHeight = getLevelConstant(requestedLevel);
        double topLevel = getLevelConstant(5);
        double bottomLevel = getLevelConstant(0);

        double currentPos = getPositionMeters();
        boolean isPositive =  ( levelHeight - currentPos >= 0 );

        // stop motors if ( negative difference -> check bottom limit OR positive difference -> check top limit )


        //if ( (!isPositive && bottomLevel <= currentPos ) || (isPositive && topLevel >= currentPos )) {
        if ( (!isPositive && UnitsUtility.isBeamBroken(bottomLimitSwitch, true, this.getName())) || (isPositive && getTopBeamBreak())) {
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
                * (1 / Constants.ElevatorConstants.ELEVATOR_GEARING) * 1.179042253521127;
    }


    public double getVelocityMetersPerSecond() {
        return (relativeEncoder.getVelocity() / 60) * (2 * Math.PI * Constants.ElevatorConstants.ELEVATOR_DRUM_RADIUS)
                * (1 / Constants.ElevatorConstants.ELEVATOR_GEARING) * 1.179042253521127;
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
//Todo: Low Priority: figure out coast and brake states for motor set

//    public void coast(){
//        SparkMaxConfig coastConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
//        elevatorMotors.configure(coastConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//
//    }
//
//    public void brake(){
//        SparkMaxConfig brakeConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);
//        elevatorMotors.motor.configure(brakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//    }

}
