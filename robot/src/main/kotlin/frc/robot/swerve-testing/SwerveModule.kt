import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.AnalogInput
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule () {
    private val CANSparkMax driveMotors
    private val CANSparkMax turningMotors

    private val CANEncoder driveEncoder
    private val CANEncoder turningEncoder

    private val PIDController turningPIDController

    private val AnalogInput absoluteEncoder
    private val booLean absoluteEncoderReversed
    private val double absoluteEncoderOffsetRad

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
        boolean turningMotorReversed, int absoluteEncoderIdea, double absoluteEncoderOffset,boolean absoluteEncoderRevesrsed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset
        this.absoluteEncoderReversed = absoluteEncoderReversed
    }









}