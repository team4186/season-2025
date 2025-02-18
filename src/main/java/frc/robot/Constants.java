// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // TODO: Update with final robot weight
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprak max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16.0);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


  // TODO: Update after testing auto
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.0, 0.0, 0.0);
  }


  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static final class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6.0;
  }


  // TODO: Update with motor constants such as MAX_SPEED, PID_CONSTANTS, ECT.
  public static final class AlgaeProcessorConstants {
    public static final double ALGAE_PROCESSOR_SWING_VOLTAGE = 5.0;
    public static final double ALGAE_PROCESSOR_INTAKE_VOLTAGE = 5.0;
    public static final double proportional = 0.05;
    public static final double integral = 0.0;
    public static final double derivative = 0.0;
  }


  public static final class ClimberConstants {
    public static final int sparkID = 0; //TODO: placeHolder
    //Climb voltage requires much higher values than move voltage.
    public static final int climbVoltage = 0; //TODO: placeHolder
    public static final int moveVoltage = 0; //TODO: placeHolder
  }


  public static final class DeAlgaeConstants {
    public static final int CanId = 0; //TODO: placeHolder
    public static final int CanId2 = 0; //TODO: placeHolder
    public static final double armDefaultAngle = 0.0; //TODO: find arm offset
    public static final double flatAngle = 0.0; //TODO: find the 'distance' of 90 degrees
    public static final double topSpeed = 5.0; // TODO: placeholder
    public static final double minSpeed = 1.0; //TODO: placeHolder
    public static final double p = 0.5 , i = 0.0, d = 0.0; //TODO: tune pid values
    public static final double maxAngle = 135; // TODO: find max arc
    public static final double minAngle = 45; // TODO: find min arc
  }


  public static final class ElevatorConstants {
    public static final double LEVEL_ONE_HEIGHT = 5; //Change the level heights here
    public static final double LEVEL_TWO_HEIGHT = 10;
    public static final double LEVEL_THREE_HEIGHT = 15;
    public static final double DEFAULT_FREE_MOVE_SPEED = 0.4;
    public static final double DEFAULT_FREE_MOVE_DOWN_SPEED = 0.1;
    public static final double DEFAULT_SETPOINT_THRESHOLD = 2.5;
    public static final double ENCODER_CONVERSION_FACTOR = 2; // CHANGE THIS!?!?!?!?! This is the value of distance/pulses
  }


  public static final class EndEffectorConstants {
    public static final int beamBreakChannel = 0; //TODO: placeHolder
    public static final double speed = 1.0; // TODO: change speed
  }


  public static final class VisionConstants {
    //update to 2025
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField( AprilTagFields.k2024Crescendo );

    // Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
    public final double maximumAmbiguity = 0.25;
  }

  public static final class ElevatorYAGSLConstants {
    public static final double ElevatorKp = 5;
    public static final double ElevatorKi = 0;
    public static final double ElevatorKd = 0;
    public static final double ElevatorMaxVelocity = 4.0;
    public static final double ElevatorMaxAcceleration = 6.0;
    public static final double ElevatorKs = 5; //voltage to overcome static friction
    public static final double ElevatorKg = 0; //voltage to overcome gravity
    public static final double ElevatorKv = 0; //velocity
    public static final double ElevatorKa = 0; //acceleration
    public static final double ElevatorRampRate = 5;
    public static final double ElevatorGearing = 0;
    public static final double ElevatorCarriageMass = 0;
    public static final double ElevatorDrumRadius = 0;
    public static final double ElevatorMinHeightMeters = 5;
    public static final double ElevatorMaxHeightMeters = 0;
  }
}
