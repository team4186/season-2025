// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

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
  public static final double ROBOT_MASS = (105.8) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprak max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16.0);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


  // TODO: Update after testing auto
  public static final class AutonConstants {
    public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(0.05, 0.0, 0.0);
    public static final PIDConstants AUTO_ANGLE_PID = new PIDConstants(0.05, 0.0, 0.0);

    // Assume the below is in feet
    public static final double DRIVE_DISTANCE = Meters.convertFrom(2.0, Feet);
    public static final double DRIVE_VELOCITY = 1.0;
  }


  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static final class OperatorConstants {
    public static final double DEADBAND = 0.09;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6.0;
  }


  // TODO: Update with motor constants such as MAX_SPEED, PID_CONSTANTS, ECT.
  public static final class AlgaeProcessorConstants {
    public static final int ALGAE_PROCESSOR_LSChannel = 0;

    public static final double ALGAE_PROCESSOR_DEFAULT_ANGLE = 5;
    public static final double ALGAE_PROCESSOR_MAX_ANGLE = 50.0;
    public static final double ALGAE_PROCESSOR_GEARBOX_RATIO = 30;
    public static final double ALGAE_PROCESSOR_HOLDING_ANGLE = 30;

    public static final double ALGAE_PROCESSOR_MAX_SPEED = 0.25;
    public static final double ALGAE_PROCESSOR_WHEEL_INTAKE_SPEED = 0.35;
    public static final double ALGAE_PROCESSOR_WHEEL_OUTPUT_SPEED = 0.5;
    public static final double ALGAE_PROCESSOR_WHEEL_HOLDING_SPEED = 0.10;
  }


  public static final class ClimberConstants {
    public static final int CLIMBER_LSChannel = 8;

    public static final double CLIMBER_SPEED = 0.9;
    public static final double CLIMBER_GEARBOX_RATIO = 75; //TODO: slightly inaccurate
    public static final double CLIMBER_DEPLOY_ANGLE = 300; //TODO: placeholder, relative to limit switch
  }


  public static final class DeAlgaeConstants {
    public static final int DE_ALGAE_LSChannel = 9; //

    public static final double DE_ALGAE_P = 0.05;
    public static final double DE_ALGAE_I = 0.0;
    public static final double DE_ALGAE_D = 0.0;

    public static final double DE_ALGAE_DEFAULT_ANGLE = 5.0;
    public static final double DE_ALGAE_MAX_SPEED = 0.2;
    public static final double DE_ALGAE_MIN_SPEED = 0.1;

    public static final double DE_ALGAE_MAX_ANGLE = 135.0;
    public static final double DE_ALGAE_GEARBOX_RATIO = 54.8;
    public static final double DE_ALGAE_WHEEL_MAX_SPEED = 1.0;
  }


  public static final class ElevatorConstants {
    public static final int ELEVATOR_BOTTOM_LIMIT_ID = 7;
    public static final int ELEVATOR_TOP_LIMIT_ID = 6;
    public static final int ELEVATOR_ENCODER_ID_A = 5;
    public static final int ELEVATOR_ENCODER_ID_B = 4;

    public static final double ELEVATOR_RAMP_RATE = 5;
    public static final double ELEVATOR_MAX_VELOCITY = 1.0;
    public static final double ELEVATOR_MAX_ACCELERATION = 1.0;

    public static final double ELEVATOR_MIN_HEIGHT = 0.0; //TODO: Update heights
    public static final double ELEVATOR_LEVEL_ONE = 0.1970; // why even consider the tray? can we score with the elevator?
    public static final double ELEVATOR_LEVEL_TWO = 0.4900; // 0.5050 old 70 cm
    public static final double ELEVATOR_LEVEL_THREE = 0.8200; // 0.86 old 118 cm
    public static final double ELEVATOR_LEVEL_FOUR = 1.4000; // 189 cm
    public static final double ELEVATOR_MAX_HEIGHT = 1.4198; // TODO: Determine threshold if different from highest level

    public static final double ELEVATOR_DEFAULT_FREE_MOVE_SPEED = 0.4;
    public static final double ELEVATOR_DEFAULT_FREE_MOVE_DOWN_SPEED = 0.1;

    //TODO: Change the two below, they are placeholders.
    public static final double ELEVATOR_GEAR_RATIO = 0.25;
    public static final double ELEVATOR_SHAFT_CIRCUMFERNCE = 2 * Math.PI * ELEVATOR_GEAR_RATIO;
    public static final double ENCODER_CONVERSION_FACTOR = ELEVATOR_SHAFT_CIRCUMFERNCE/(42.0/ELEVATOR_GEAR_RATIO); // CHANGE THIS!?!?!?!?! This is the value of distance/pulses

    public static final double ELEVATOR_GEARING = 12.0; // TODO: Update with gear ratio
    public static final double ELEVATOR_CARRIAGE_MASS = 4.0; // end effector mass, with / without
    public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(1.0);

    // TODO: Update values
    /**
     * Previous values:
     *   P = 8.0
     *   I and D are 0
     */
    public static final double ELEVATOR_P = 7.5;
    public static final double ELEVATOR_I = 0.5;
    public static final double ELEVATOR_D = 0.0;

    // Adjust these to reach optimal
    public static final double ELEVATOR_KS = 0.0; // Static gain in volts
    public static final double ELEVATOR_KG = 0.0; // Gravity gain in volts
    public static final double ELEVATOR_KV = 0.0; // Velocity gain in V/(m/s)
    public static final double ELEVATOR_KA = 0.0; // Acceleration gain in V/(m/s^2)

    public static final double ELEVATOR_DEFAULT_TOLERANCE = 0.0075; // Meters (+/-)Tolerance
  }


  public static final class EndEffectorConstants {
    public static final int END_EFFECTOR_BEAM_BREAK = 3;

    public static final double END_EFFECTOR_EJECT_SPEED_ADJ = 0.450;
    public static final double END_EFFECTOR_EJECT_SPEED = 0.475;
    // public static final double END_EFFECTOR_EJECT_SPEED_L4 = 0.55;
    public static final double END_EFFECTOR_INTAKE_SPEED = 0.25;
  }


  public static final class VisionConstants {
    // Where is this used? It's defined in SwerveSubsystem but never used anywhere.
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField( AprilTagFields.k2025ReefscapeWelded);

    // Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
    public static final double MAXIMUM_AMBIGUITY = 0.25;

    public static final String LIME_LIGHT_NAME = "limelight";

    // Offset from april tag center for scoring on the right.
    // Change number later.
    public static final double LEFT_SCORE_OFFSET =  0.11;
    public static final double RIGHT_SCORE_OFFSET =  0.47 ;
    // Get to tuning
    public static final double ANGLE_P = 0.03;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;

    public static final double STRAFE_P = 1.8; // 1.6 old
    public static final double STRAFE_I = 0.0;
    public static final double STRAFE_D = 0.0;

    public static final double DISTANCE_P = 0.25;
    public static final double DISTANCE_I = 0.0;
    public static final double DISTANCE_D = 0.0;

    // More constants to tune *crying*
    // TODO: set tolerance as well these are some default values to test (maybe it works, maybe it doesn't)
    public static final double TURN_TOLERANCE  = 1.0;
    public static final double DISTANCE_TOLERANCE = 0.05;
    public static final double STRAFE_TOLERANCE = 0.05;


    // Change this (it is in half foot)
    public static final double BUFFER_DIST = 2.0;
  }
}
