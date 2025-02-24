// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.actions.AlignToTargetCommand;
import frc.robot.commands.actions.DeAlgaeCommand;
import frc.robot.commands.actions.ElevatorCommand;
import frc.robot.commands.actions.EndEffectorCommand;
import frc.robot.hardware.LimeLightRunner;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.subsystems.*;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandJoystick joystick = new CommandJoystick(0);
  private final Components motorComponents = Components.getInstance();


  // The robot's subsystems defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(
          new File( Filesystem.getDeployDirectory(), "swerve/team4186") );



  // TODO: Implement Components
  // private final Climber climber = new Climber();
  // private final AlgaeProcessor algaeProcessor = new AlgaeProcessor();

  private final EndEffector endEffector = new EndEffector(
          motorComponents.endEffectorMotor
          //new DigitalInput(Constants.EndEffectorConstants.END_EFFECTOR_BEAM_BREAK)
  );

  // Elevator( bottomLimit, topLimit, motorSet, thru_bore_encoder, pid );
//  private final Elevator elevator = new Elevator(
//          new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_LIMIT_ID),
//          new DigitalInput(Constants.ElevatorConstants.ELEVATOR_TOP_LIMIT_ID),
//          motorComponents.elevatorMotors,
//          // Defaults to 4X decoding and non-inverted (4x expected to cause jitters!)
//          new Encoder(
//                  Constants.ElevatorConstants.ELEVATOR_ENCODER_ID,
//                  Constants.ElevatorConstants.ELEVATOR_ENCODER_ID,
//                  false,
//                  CounterBase.EncodingType.k1X),
//          new PIDController(
//                  Constants.ElevatorConstants.ELEVATOR_P,
//                  Constants.ElevatorConstants.ELEVATOR_I,
//                  Constants.ElevatorConstants.ELEVATOR_D
////                  new TrapezoidProfile.Constraints(
////                          Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY,
////                          Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
//          ),
//          new ElevatorFeedforward(
//                  Constants.ElevatorConstants.ELEVATOR_KG,
//                  Constants.ElevatorConstants.ELEVATOR_KV,
//                  Constants.ElevatorConstants.ELEVATOR_KA)
//  );


//  private final Climber climber = new Climber(
//          Components.getInstance().climberMotor,
//          new DigitalInput(Constants.ClimberConstants.TFChannel),
//          new PIDController(
//                  Constants.ClimberConstants.PROPORTIONAL,
//                  Constants.ClimberConstants.INTEGRAL,
//                  Constants.ClimberConstants.DERIVATIVE),
//          Constants.ClimberConstants.TARGETANGLE,
//          Constants.ClimberConstants.MAXVOLTS,
//          Constants.ClimberConstants.MINVOLTS
//  );
//

//  private final DeAlgae deAlgae = new DeAlgae(
//          motorComponents.deAlgaeWheelMotor,
//          motorComponents.deAlgaeAngleMotor,
//          new PIDController(
//                  Constants.DeAlgaeConstants.DE_ALGAE_P,
//                  Constants.DeAlgaeConstants.DE_ALGAE_I,
//                  Constants.DeAlgaeConstants.DE_ALGAE_D));

  // TODO: Uncomment below later.
  private final LimeLightRunner visionSubsystem = new LimeLightRunner();


  /**
   * Commands are implemented here...
   */
  AlignToTargetCommand alignCommand = new AlignToTargetCommand(
          visionSubsystem,
          drivebase,
          // ignore below offset may not be needed.
          new Translation2d(0.0,0.0)
  );


  EndEffectorCommand endEffectorCommand = new EndEffectorCommand(endEffector);

/**
 * Elevator commands
 */
//  ElevatorCommand elevatorCommandL1 = new ElevatorCommand(
//        elevator,
//        1);
//
//  ElevatorCommand elevatorCommandL2 = new ElevatorCommand(elevator,
//  2);
//
//  ElevatorCommand elevatorCommandL3 = new ElevatorCommand(
//          elevator, // elevatorsubsystem
//          3 // level
//          );
//
//  ElevatorCommand elevatorCommandL4 = new ElevatorCommand(elevator,
//  4);
//
//  DeAlgaeCommand deAlgaeCommand = new DeAlgaeCommand(deAlgae);


//  // Conditions to be met: In front of target april tag
//  ScoreCorralCommand scoreCorralCommand = new ScoreCorralCommand(
//          // vision
//          // swervesubsystem
//          // elevator
//          // end effector
//          // left, right offset
//          // level
//  );


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
                  () -> attenuated( joystick.getY(), 2, 1.0 ) * -1,
                  () -> attenuated( joystick.getX(), 2, 1.0 ) * -1)
          .withControllerRotationAxis(
                  () -> attenuated( joystick.getTwist(), 3, 0.9 ) * -1)
          .deadband(OperatorConstants.DEADBAND)
          .allianceRelativeControl(true);

  // Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  // NOTE: No need to change heading axis with joystick
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
          .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
                  () -> -joystick.getY(),
                  () -> -joystick.getX())
          .withControllerRotationAxis(
                  () -> joystick.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(
          () -> Math.sin( joystick.getRawAxis(2) * Math.PI ) * ( Math.PI * 2 ),
          () -> Math.cos( joystick.getRawAxis(2) * Math.PI ) * ( Math.PI * 2))
          .headingWhile(true);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if ( Robot.isSimulation() ){
      // override to sim controls
      // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      // joystick.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // NOTE: Change later?
      joystick.trigger().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(10, 3, new Rotation2d()))));
      joystick.button(11).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    if ( DriverStation.isTest() ){
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      joystick.button(2).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly() );
      joystick.button(3).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      joystick.button(5).whileTrue(drivebase.centerModulesCommand());
      joystick.button(6).onTrue(Commands.none());

      // Example Align Command Object
      joystick.button(10).whileTrue(alignCommand);
      // AlgaeProcessor Tests
      /**
       * Extend
       * Retract
       * Intake Algae
       * Eject Algae
       */

      // Climber Tests
      /**
       * Extend
       * Latch
       * Retract
       */

      // DeAlgae Tests
      /**
       * Extend
       * Remove Algae (up)
       * Remove Algae (down)
       */

      /*
      runs deAlgaeCommand when button is held down.
      Is interrupted when let go and automatically moves back to default position
      and stops rolling motor.
      */
      // joystick.button(7).whileTrue(deAlgaeCommand);

      // Elevator Tests
      /**
       * Level 1, 2, 3
       * Limit switch (upper, lower)
       */

      // EndEffector Tests
      /**
       * Intake / Eject Coral
       */

      joystick.trigger().whileTrue(endEffectorCommand);


    } else {
      joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // joystick.button(0).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      joystick.button(9).whileTrue(
              drivebase.driveToPose(
                      new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      joystick.button(10).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // joystick.button(0).onTrue(Commands.none());
    }
  }


  // Adjust joystick input from linear to exponential curve
  private double attenuated(double value, int exponent, double scale) {
    double res = scale * Math.pow( Math.abs(value), exponent );
    if ( value < 0 ) { res *= -1; }
    return res;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }


  public void setMotorBrake(boolean brake) { drivebase.setMotorBrake(brake); }
}
