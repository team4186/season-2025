package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
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

    // Limit Switches
    this.bottomLimitSwitch = bottomLimitSwitch;
    this.topLimitSwitch = topLimitSwitch;

    this.encoder = encoder;
    this.encoder.setDistancePerPulse( 1.06938/6626.506 );
    this.encoder.setMinRate(0.0075); // MetersPerSecond

    // control
    this.pid = pid;
    this.pid.reset(0.0);
    this.elevatorFeedforward = elevatorFeedforward;

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
    SmartDashboard.putNumber("Elevator_BoreEncoder_Distance", encoder.getDistance());
    SmartDashboard.putNumber("Elevator_BoreEncoder_Rate", encoder.getRate());
    SmartDashboard.putNumber("Elevator_Requested_Velocity", Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY);

    SmartDashboard.putNumber("Elevator_Acceleration_Expected", Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION);

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
  public void elevatorZeroFailsafe(double speed){
    elevatorMotors.setLeadSpeed(speed);
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
    return encoder.getDistance();
  }


  public double getVelocityMetersPerSecond() {
    return encoder.getRate();
  }


  public boolean isAtTop() {
    return topLimitSwitch.get();
  }


  public boolean isAtBottom() {
    return bottomLimitSwitch.get();
    //Got rid of an exclamation point before the bottomLimitSwitch.get() so might need to put it back. Did this as limit switch was invesrsed.
  }


  public void stopMotor() {
    elevatorMotors.stop();
  }


  // Todo: Tune this value
  public void slowResetToBottomLimitSwitch(){
    if ( isAtBottom() ){
      // add updated bottom with limit switch hit?
      encoder.reset();
      stopMotor();
    } else {
      elevatorMotors.setLeadVoltage(
          Constants.ElevatorConstants.ELEVATOR_KG - Constants.ElevatorConstants.ELEVATOR_KS
          // - Constants.ElevatorConstants.ELEVATOR_KG
      );
    }
  }


  // Simple brake setting for
  public Runnable applyVoltage(double voltage, boolean isPositive) {
    if (isPositive){
      return () -> elevatorMotors.setLeadVoltage( voltage );
    } else {
      return () -> elevatorMotors.setLeadVoltage( voltage * -1 );
    }
  }
}
