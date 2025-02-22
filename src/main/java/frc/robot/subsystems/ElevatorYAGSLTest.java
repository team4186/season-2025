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
import frc.robot.sparkmaxconfigs.ElevatorMotorSet;
import frc.robot.sparkmaxconfigs.MotorSet;
import frc.robot.Units;

public class ElevatorYAGSLTest extends SubsystemBase {
    private DCMotor elevatorGearbox = DCMotor.getNEO(2);
    private final ElevatorMotorSet elevatorMotorsYagsl = Components.getInstance().elevatorMotorsYagsl; //we are using multiple motors
    private final RelativeEncoder encoder = elevatorMotorsYagsl.getLeadEncoder();
    private final ProfiledPIDController pidController =
            new ProfiledPIDController(
                    Constants.ElevatorYAGSLConstants.ElevatorKp,
                    Constants.ElevatorYAGSLConstants.ElevatorKi,
                    Constants.ElevatorYAGSLConstants.ElevatorKd,
                    new TrapezoidProfile.Constraints(
                            Constants.ElevatorYAGSLConstants.ElevatorMaxVelocity,
                            Constants.ElevatorYAGSLConstants.ElevatorMaxAcceleration
                    )
            );

    private final ElevatorFeedforward feedForward = new ElevatorFeedforward(
            Constants.ElevatorYAGSLConstants.ElevatorKg,
            Constants.ElevatorYAGSLConstants.ElevatorKv,
            Constants.ElevatorYAGSLConstants.ElevatorKs);

    private final DigitalInput limitSwitchLow = new DigitalInput(0);


    public double distanceInMeters() {
        // Change if not NEO
        return Units.TicksToMeters(encoder.getPosition(),
                Constants.ElevatorYAGSLConstants.ElevatorDrumRadius,
                Constants.ElevatorYAGSLConstants.ElevatorGearing);
    }

    public double getVelocityMetersPerSecond() {
        // Ignore the below placeholder.
        return 0.0;
        // return Units.TicksToMeters(encoder.getVelocity().)   <-- Unfinished.
    }
}
