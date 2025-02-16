package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;



public class AlgaeProcessor extends SubsystemBase {
    // ReadyPosition in degrees
    // 90 is placeholder, change when testing.
    private static final double readyPos = 90;
    // Placeholder CanID (Not actually 1).
    private static final int CANIDSwing = 1;
    // Placeholder CAN
    private static final int CANIDIntake = 2;

    private static final SparkMax angleMotor = new SparkMax(CANIDSwing, SparkLowLevel.MotorType.kBrushless);

    private static final SparkMax intakeMotor = new SparkMax(CANIDIntake, SparkLowLevel.MotorType.kBrushless);

    public Command intakeAlgae() {
        // Should move to ready position and intake algae.
    }

    public Command launchAlgae() {
        // Should move the motors to launch algae.
    }

    public Command moveToSafePos() {
        // Should move the arm to inside frame perimeter.
    }
}
