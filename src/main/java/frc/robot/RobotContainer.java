// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.actions.AlgaeProcessorCommand;
import frc.robot.commands.actions.AlignToReefCommand;
import frc.robot.commands.actions.AlignToTargetCommand;
import frc.robot.commands.actions.ClimberCommand;
import frc.robot.commands.actions.DeAlgaeCommand;
import frc.robot.commands.actions.ElevatorCommand;
import frc.robot.commands.actions.ElevatorReturnToDefault;
import frc.robot.commands.actions.EndEffectorEjectCommand;
import frc.robot.commands.actions.EndEffectorLoadCommand;
import frc.robot.commands.actions.ManualElevatorCommand;
import frc.robot.hardware.LimeLightRunner;
import frc.robot.sparkmaxconfigs.Components;
import frc.robot.subsystems.AlgaeProcessor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DeAlgae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Controller and components
  final CommandJoystick joystickDriver = new CommandJoystick(0);
  final CommandJoystick joystickOperator = new CommandJoystick(1);

  final CommandXboxController xboxController = new CommandXboxController(0);

  private final Components motorComponents = Components.getInstance();
  SendableChooser<Command> autoChooser = new SendableChooser<>();


  // Robot Subsystems
  private final LimeLightRunner visionSubsystem = new LimeLightRunner(false);

  private final SwerveSubsystem drivebase  = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/team4186"), false, visionSubsystem);

  private final Elevator elevator = new Elevator(
      new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_LIMIT_ID),
      new DigitalInput(Constants.ElevatorConstants.ELEVATOR_TOP_LIMIT_ID),
      motorComponents.elevatorMotorSet,
      // Defaults to 4X decoding and non-inverted (4x expected to cause jitters!)
      new Encoder(
          Constants.ElevatorConstants.ELEVATOR_ENCODER_ID_A,
          Constants.ElevatorConstants.ELEVATOR_ENCODER_ID_B,
          false,
          CounterBase.EncodingType.k1X),
      new ProfiledPIDController(
          Constants.ElevatorConstants.ELEVATOR_P,
          Constants.ElevatorConstants.ELEVATOR_I,
          Constants.ElevatorConstants.ELEVATOR_D,
          new TrapezoidProfile.Constraints(
              Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY,
              Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION)),
      new ElevatorFeedforward(
          Constants.ElevatorConstants.ELEVATOR_KS,
          Constants.ElevatorConstants.ELEVATOR_KG,
          Constants.ElevatorConstants.ELEVATOR_KV,
          Constants.ElevatorConstants.ELEVATOR_KA)
  );

  private final EndEffector endEffector = new EndEffector(
      motorComponents.endEffectorSingleMotor,
      new DigitalInput(Constants.EndEffectorConstants.END_EFFECTOR_BEAM_BREAK)
  );

  private final DeAlgae deAlgae = new DeAlgae(
      motorComponents.deAlgaeWheelSingleMotor,
      motorComponents.deAlgaeAngleSingleMotor,
      new PIDController(
          Constants.DeAlgaeConstants.DE_ALGAE_P,
          Constants.DeAlgaeConstants.DE_ALGAE_I,
          Constants.DeAlgaeConstants.DE_ALGAE_D),
      new DigitalInput(Constants.DeAlgaeConstants.DE_ALGAE_LSChannel)

  );

  private final AlgaeProcessor algaeProcessor = new AlgaeProcessor(
      motorComponents.algaeProcessorWheelSingleMotor,
      motorComponents.algaeProcessorAngleSingleMotor,
      new DigitalInput(Constants.AlgaeProcessorConstants.ALGAE_PROCESSOR_LSChannel)
  );

  private final Climber climber = new Climber(
      motorComponents.climberSingleMotor,
      new DigitalInput(Constants.ClimberConstants.CLIMBER_LSChannel)
  );


  // Robot Commands
  AlignToTargetCommand alignCommand = new AlignToTargetCommand(
      visionSubsystem,
      drivebase,
      new PIDController(Constants.VisionConstants.ANGLE_P,
          Constants.VisionConstants.ANGLE_I,
          Constants.VisionConstants.ANGLE_D),
      new PIDController(Constants.VisionConstants.STRAFE_P,
          Constants.VisionConstants.STRAFE_I,
          Constants.VisionConstants.STRAFE_D),
      new PIDController(Constants.VisionConstants.DISTANCE_P,
          Constants.VisionConstants.DISTANCE_I,
          Constants.VisionConstants.DISTANCE_D)
  );

  AlignToReefCommand alignToReefLeftCommand = new AlignToReefCommand(
      true,
      visionSubsystem,
      drivebase,
      new PIDController(Constants.VisionConstants.ANGLE_P,
          Constants.VisionConstants.ANGLE_I,
          Constants.VisionConstants.ANGLE_D),
      new PIDController(Constants.VisionConstants.STRAFE_P,
          Constants.VisionConstants.STRAFE_I,
          Constants.VisionConstants.STRAFE_D),
      new PIDController(Constants.VisionConstants.DISTANCE_P,
          Constants.VisionConstants.DISTANCE_I,
          Constants.VisionConstants.DISTANCE_D)
  );

  AlignToReefCommand alignToReefRightCommand = new AlignToReefCommand(
      false,
      visionSubsystem,
      drivebase,
      new PIDController(
          Constants.VisionConstants.ANGLE_P,
          Constants.VisionConstants.ANGLE_I,
          Constants.VisionConstants.ANGLE_D),
      new PIDController(
          Constants.VisionConstants.STRAFE_P,
          Constants.VisionConstants.STRAFE_I,
          Constants.VisionConstants.STRAFE_D),
      new PIDController(
          Constants.VisionConstants.DISTANCE_P,
          Constants.VisionConstants.DISTANCE_I,
          Constants.VisionConstants.DISTANCE_D)
  );


  EndEffectorEjectCommand endEffectorEjectCommand = new EndEffectorEjectCommand(endEffector, false);
  EndEffectorEjectCommand endEffectorEjectCommandSlow = new EndEffectorEjectCommand(endEffector, true);
  EndEffectorLoadCommand endEffectorLoadCommand = new EndEffectorLoadCommand(endEffector);


  DeAlgaeCommand deAlgaeCommand = new DeAlgaeCommand(deAlgae);
  AlgaeProcessorCommand algaeProcessorCommand = new AlgaeProcessorCommand(algaeProcessor);


  ClimberCommand climberCommand = new ClimberCommand(climber);


  ElevatorReturnToDefault elevatorDefaultCommand = new ElevatorReturnToDefault(elevator);
  ElevatorCommand elevatorCommandL1 = new ElevatorCommand(elevator, 1);
  ElevatorCommand elevatorCommandL2 = new ElevatorCommand(elevator, 2);
  ElevatorCommand elevatorCommandL3 = new ElevatorCommand(elevator, 3);
  ElevatorCommand elevatorCommandL4 = new ElevatorCommand(elevator, 4);


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> attenuated( joystickDriver.getY(), 2, 1.0 ) * 1,
          () -> attenuated( joystickDriver.getX(), 2, 1.0 ) * 1)
      .withControllerRotationAxis(
          () -> attenuated( joystickDriver.getTwist(), 2, 0.75 ) * 1)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> attenuated( joystickDriver.getY(), 2, 0.5 ) * 1,
          () -> attenuated( joystickDriver.getX(), 2, 0.5 ) * 1)
      .withControllerRotationAxis(
          () -> attenuated( joystickDriver.getTwist(), 2, 0.375 ) * 1)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityXBox = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> xboxController.getLeftY() * -1,
          () -> xboxController.getLeftX() * -1)
      .withControllerRotationAxis(xboxController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  // NOTE: No need to change heading axis with joystick
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> -joystickDriver.getY(),
          () -> -joystickDriver.getX())
      .withControllerRotationAxis(
          () -> joystickDriver.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(
          () -> Math.sin( joystickDriver.getRawAxis(2) * Math.PI ) * ( Math.PI * 2 ),
          () -> Math.cos( joystickDriver.getRawAxis(2) * Math.PI ) * ( Math.PI * 2))
      .headingWhile(true);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    /* TODO: Register Named Commands used during PathPlanner
     *   Update pathplanner to new field layout
     *   Note: Care must be taken with commands that might conflict w.r.t. required subsystems and pathplanner
     * */
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    new EventTrigger("Say hello").onTrue(Commands.print("Hello world"));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    /**
     * Make auto commands here.
     */
    final Command placeholderAuto = drivebase.driveToDistanceCommand(10, 0.5);

    /**
     * Auto Selection
     */
    autoChooser.setDefaultOption("No pill", placeholderAuto);
    autoChooser.addOption("Red pill", placeholderAuto);
    autoChooser.addOption("Blue pill", placeholderAuto);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAngularVelocitySlow = drivebase.driveFieldOriented(driveAngularVelocitySlow);
    // Command driveFieldOrientedAngularVelocityWithPov = drivebase.driveFieldOriented();


    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);


    // Set default subsystem commands here
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // TODO: Uncomment and test after FF set
    elevator.setDefaultCommand( elevatorDefaultCommand );
    // elevator.setDefaultCommand( Commands.runOnce( elevator::reset, elevator ).repeatedly());


    if (Robot.isSimulation()){
      // override to sim controls
      // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      joystickDriver.button(3).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      joystickDriver.button(4).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 6, new Rotation2d()))));
      joystickDriver.button(5).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(6, 3, new Rotation2d()))));
      joystickDriver.button(6).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(6, 6, new Rotation2d()))));
    }

    if ( DriverStation.isTest() ) {
      // Test commands go here
      drivebase.setDefaultCommand(Commands.none());
      elevator.setDefaultCommand(Commands.none());

      joystickOperator.trigger().onTrue(endEffectorEjectCommandSlow);
      joystickDriver.trigger().onTrue(endEffectorEjectCommand);
      //      joystick.button(2).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly() );
      //      joystick.button(3).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));

//            joystickDriver.button(3).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
//
//            joystickDriver.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
//            joystickDriver.button(5).whileTrue(drivebase.centerModulesCommand());
//            joystickDriver.button(10).onTrue( Commands.runOnce( drivebase::lock, drivebase ));
//
//            joystickDriver.button(11).onTrue( drivebase.setMotorBrakeCommand(true) );
//            joystickDriver.button(12).onTrue( drivebase.setMotorBrakeCommand(false) );
//
//            joystickDriver.trigger().onTrue( drivebase.driveToDistanceCommand(Constants.AutonConstants.DRIVE_DISTANCE, Constants.AutonConstants.DRIVE_VELOCITY) );


      // TESTING FOR FF ELEVATOR SysIdRoutine -> SENSITIVE!!!
//                    joystickDriver.button(5).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//                    joystickDriver.button(3).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//            //
//                    joystickDriver.button(6).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
//                    joystickDriver.button(4).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//
//                    // Voltage Down
//                    joystickDriver.button(11).whileTrue( Commands.runOnce(elevator.applyVoltage(7, false)).repeatedly());
//
//                    // Voltage Up
//                    joystickDriver.button(9).whileTrue( Commands.runOnce(elevator.applyVoltage(7, true)).repeatedly());

    } else {

      /// Useful swerve commands
//            joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
//            joystick.button(5).whileTrue(drivebase.centerModulesCommand());

      // EndEffector
      joystickOperator.trigger().onTrue(endEffectorEjectCommandSlow);
      joystickDriver.trigger().onTrue(endEffectorEjectCommand);

      joystickDriver.button(2).onTrue(endEffectorLoadCommand);
      joystickOperator.button(2).onTrue(endEffectorLoadCommand);
//            joystickDriver.trigger().whileTrue(Commands.runOnce(endEffector::eject, endEffector).repeatedly());
//            joystickDriver.button(2).whileTrue(Commands.runOnce(endEffector::intake, endEffector).repeatedly());

      // Algae - Cycle State on button press
      joystickOperator.button(3).onTrue(algaeProcessorCommand);
      joystickOperator.button(3).onTrue((Commands.runOnce(algaeProcessorCommand::button_detect)));

      joystickOperator.button(5).onTrue(deAlgaeCommand);

      // Climber
      joystickOperator.button(6).onTrue(climberCommand);
      joystickOperator.button(6).onTrue((Commands.runOnce(climberCommand::button_detect)));


      // Elevator - Go to level and maintain
      joystickOperator.button(7).whileTrue(elevatorCommandL1);
      joystickOperator.button(8).whileTrue(elevatorCommandL2);
      joystickOperator.button(9).whileTrue(elevatorCommandL3);
      // Uncomment below when testing.
      // joystickOperator.button(11).whileTrue(alignCommand);
      // joystickOperator.button(10).whileTrue(elevatorCommandL4);
      joystickOperator.button(12).whileTrue((Commands.runOnce(elevator::slowResetToBottomLimitSwitch, elevator).repeatedly()));

      // Joystick Operator strafing here for buttons 11 and 12
      joystickDriver.button(11).whileTrue(driveFieldOrientedAngularVelocitySlow);
      joystickDriver.button(7).onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

//             TODO: Uncomment this and test align to reef
//                 joystickOperator.button(10).whileTrue(alignToReefLeftCommand);
//                 joystickOperator.button(11).whileTrue(alignToReefRightCommand);
//             TODO: Uncomment this and test align to target (general version)
//                 joystickOperator.button(12).whileTrue(alignCommand);
//
//             joystickDriver.button(8).onTrue(Commands.runOnce(drivebase::zeroGyro));
//
//             joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
//
//             joystick.button(0).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
//             joystick.button(9).whileTrue(
//                     drivebase.driveToPose(
//                            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
//             joystick.button(10).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//             joystick.button(0).onTrue(Commands.none());
//
//
//             deAlgae Command testing
//             joystick.button(7).onTrue(deAlgaeCommand);
//             joystick.button(7).onTrue((Commands.runOnce(deAlgaeCommand::button_detect)));


//            joystick.button(8).whileTrue(Commands.runOnce(deAlgae::Manpid_runMotor_Up).repeatedly())
//                  .onFalse(Commands.runOnce(deAlgae::stop));
//            joystick.button(10).whileTrue(Commands.runOnce(deAlgae::Manpid_reset).repeatedly())
//                  .onFalse(Commands.runOnce(deAlgae::stop));
//            joystick.button(11).whileTrue(Commands.runOnce(deAlgae::stop).repeatedly());
//            joystick.button(12).onTrue(Commands.runOnce(deAlgae::resetEncoder));
//            joystick.button(3).whileTrue(Commands.runOnce(deAlgae::coast)).
//                  onFalse(Commands.runOnce(deAlgae::brake));



//                  joystick.button( 2 )
//                          .whileTrue(endEffectorLoadCommand);
//                  joystick.trigger()
//                          .whileTrue(endEffectorEjectCommand);
//
//                  // Elevator Simple GOTO level commands for testing
//                  joystick.button(9).whileTrue( elevatorCommandL1 );
//                  joystick.button(10).whileTrue( elevatorCommandL2 );
//                  joystick.button(11).whileTrue( elevatorCommandL3 );
//                  joystick.button(12).whileTrue( elevatorCommandL4 );
//
//                  joystick.button( 2 )
//                          .whileTrue(endEffectorLoadCommand);
//                  joystick.trigger()
//                          .whileTrue(endEffectorEjectCommand);
//
//             Elevator Simple GOTO level commands for testing
//                  joystick.button(9).whileTrue( elevatorCommandL1 );
//                  joystick.button(10).whileTrue( elevatorCommandL2 );
//                  joystick.button(11).whileTrue( elevatorCommandL3 );
//                  joystick.button(12).whileTrue( elevatorCommandL4 );
    }

    // AlgaeProcessor testing
//        joystick.button(5).whileTrue(Commands.runOnce(algaeProcessor::runMotor_Up).repeatedly())
//              .onFalse(Commands.runOnce(algaeProcessor::stop));
//        joystick.button(6).whileTrue(Commands.runOnce(algaeProcessor::runMotor_Down).repeatedly())
//              .onFalse(Commands.runOnce(algaeProcessor::stop));
//        joystick.button(7).whileTrue(Commands.runOnce(algaeProcessor::eject).repeatedly())
//              .onFalse(Commands.runOnce(algaeProcessor::stop));
//        joystick.button(8).whileTrue(Commands.runOnce(algaeProcessor::intake).repeatedly())
//              .onFalse(Commands.runOnce(algaeProcessor::stop));
//        joystick.button(12).onTrue(Commands.runOnce(algaeProcessor::resetEncoder));
//        joystick.button(4).whileTrue(Commands.runOnce(algaeProcessor::coast)).
//              onFalse(Commands.runOnce(algaeProcessor::brake));
//        joystick.button(3).onTrue(algaeProcessorCommand);
//        joystick.button(3).onTrue((Commands.runOnce(algaeProcessorCommand::button_detect)));

//         Climber testing
//            joystick.button(5).whileTrue(Commands.runOnce(climber::runMotor_Up).repeatedly())
//                    .onFalse(Commands.runOnce(climber::stop));
//            joystick.button(6).whileTrue(Commands.runOnce(climber::runMotor_Down).repeatedly())
//                    .onFalse(Commands.runOnce(climber::stop));
//            joystick.button(5).whileTrue(Commands.runOnce(climber::coast)).
//                    onFalse(Commands.runOnce(climber::brake));
//
//
//         CLIMBER
//            joystick.button(8).onTrue(climberCommand);
//            joystick.button(8).onTrue((Commands.runOnce(climberCommand::button_detect)));
//
//         AlignToTarget testing
//         joystick.button(6).whileTrue(Commands.runOnce(AlignToTarget).repeatedly());
  }


  // Adjust joystick input from linear to exponential curve
  private double attenuated(double value, int exponent, double scale) {
    double res = scale * Math.pow(Math.abs(value), exponent);
    if (value < 0) {
      res *= -1;
    }
    return res;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    // return drivebase.getAutonomousCommand("New Auto");
    // TODO: Update with AutoCommand when implemented
    return drivebase.driveToDistanceCommand(Constants.AutonConstants.DRIVE_DISTANCE, Constants.AutonConstants.DRIVE_VELOCITY);
    // return drivebase.getAutonomousCommand("Auto 1");
  }

  /**
   * TODO:
   * @param brake
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
