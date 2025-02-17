// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.DeAlgae;
import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // final CommandXboxController joystick = new CommandXboxController(0);
  final CommandJoystick joystick = new CommandJoystick(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(
          new File( Filesystem.getDeployDirectory(), "swerve/team4186") );

  //TODO: deAlgae stuff
  DeAlgae deAlgae = new DeAlgae();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
                  () -> attenuated( joystick.getY() ) * -1,
                  () -> attenuated( joystick.getX() ) * -1)
          .withControllerRotationAxis(
                  () -> attenuated( joystick.getTwist() ))
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

    if ( RobotBase.isSimulation() ){
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if ( Robot.isSimulation() ){
      // joystick.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // NOTE: Change later?
      joystick.trigger().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      joystick.button(11).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    if ( DriverStation.isTest() ){
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      joystick.button(2).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      joystick.button(3).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      joystick.button(5).whileTrue(drivebase.centerModulesCommand());
      joystick.button(6).onTrue(Commands.none());

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

      // Elevator Tests
      /**
       * Level 1, 2, 3
       * Limit switch (upper, lower)
       */

      // EndEffector Tests
      /**
       * Intake / Eject Coral
       */


    } else {
      joystick.button(4).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // joystick.button(0).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      joystick.button(9).whileTrue(
              drivebase.driveToPose(
                      new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      joystick.button(10).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // joystick.button(0).onTrue(Commands.none());

      //TODO: deAlgae commands config buttons later
      //TODO: Vision needs to tell DeAlgae whether the roller should be inverted
      //TODO: alternatively could manually decide
//      joystick.button(5).whileTrue(Commands.runOnce(deAlgae::runMotor_inverted, deAlgae).repeatedly());
//
//      joystick.button(5).whileTrue(Commands.runOnce(deAlgae::runMotor, deAlgae).repeatedly());
//
//      joystick.button(5).whileFalse(Commands.runOnce(deAlgae::stop));
    }
  }


  // Adjust joystick input from linear to exponential curve
  private double attenuated(double value) {
    double res = 0.90 * Math.pow( Math.abs(value), 2 );
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
