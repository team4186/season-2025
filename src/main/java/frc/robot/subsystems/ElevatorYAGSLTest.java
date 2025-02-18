package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.MotorSet;
import frc.robot.sparkmaxconfigs.SingleMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints

public class ElevatorYAGSLTest extends SubsystemBase {
    private final MotorSet elevatorMotors = Components.getInstance().elevatorMotors; //we are using multiple motors
    private final SparkMax elevatorMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxSim elevatorMotorSim = new SparkMaxSim(elevatorMotors, elevatorGearbox);
    private final ProfiledPIDController pidController =
            new ProfiledPIDController(ElevatorKp, ElevatorKi, ElevatorKd,
                    new TrapezoidProfile.Constraints(ElevatorMaxVelocity, ElevatorMaxAcceleration));

    private final ElevatorFeedforward feedForward = new ElevatorFeedforward()

    private static final double ElevatorKp = 5;
    private static final double ElevatorKi = 0;
    private static final double ElevatorKd = 0;
    private static final double ElevatorMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    private static final double ElevatorMaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);
    private static final double ElevatorKs = 5; //voltage to overcome static friction
    private static final double ElevatorKg = 0; //voltage to overcome gravity
    private static final double ElevatorKv = 0; //velocity
    private static final double ElevatorKa = 0; //acceleration
    private static final double ElevatorRampRate = 5;
    private static final double ElevatorGearing = 0;
    private static final double ElevatorCarriageMass = 0;
    private static final double ElevatorDrumRadius = 0;
    private static final double ElevatorMinHeightMeters = 5;
    private static final double ElevatorMaxHeightMeters = 0;
}

