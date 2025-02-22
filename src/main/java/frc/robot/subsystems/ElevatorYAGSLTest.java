package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.sparkmaxconfigs.YagslElevatorMotorSet;
import frc.robot.sparkmaxconfigs.MotorSet;
import frc.robot.Units;

public class ElevatorYAGSLTest extends SubsystemBase {
    private DCMotor elevatorGearbox = DCMotor.getNEO(2);
    private final YagslElevatorMotorSet elevatorMotorsYagsl = Components.getInstance().elevatorMotorsYagsl; //we are using multiple motors
    private final RelativeEncoder encoder = elevatorMotorsYagsl.getLeadEncoder();
    private final ProfiledPIDController pidController =
            new ProfiledPIDController(
                    Constants.ElevatorYAGSLConstants.ELEVATOR_P,
                    Constants.ElevatorYAGSLConstants.ELEVATOR_I,
                    Constants.ElevatorYAGSLConstants.ELEVATOR_D,
                    new TrapezoidProfile.Constraints(
                            Constants.ElevatorYAGSLConstants.ELEVATOR_MAX_VELOCITY,
                            Constants.ElevatorYAGSLConstants.ELEVATOR_MAX_ACCELERATION
                    )
            );

    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
            Constants.ElevatorYAGSLConstants.ELEVATOR_G,
            Constants.ElevatorYAGSLConstants.ELEVATOR_V,
            Constants.ElevatorYAGSLConstants.ELEVATOR_S);

    private final DigitalInput limitSwitchLow = new DigitalInput(0);


    public double distanceInMeters() {
        // Change if not NEO
        return Units.TicksToMeters(encoder.getPosition(),
                Constants.ElevatorYAGSLConstants.ELEVATOR_DRUM_RADIUS,
                Constants.ElevatorYAGSLConstants.ELEVATOR_GEARING);
    }

    public double getVelocityMetersPerSecond() {
        // Ignore the below placeholder.
        return 0.0;
        // return Units.TicksToMeters(encoder.getVelocity().)   <-- Unfinished.
    }
}
