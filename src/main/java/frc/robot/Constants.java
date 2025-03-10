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
  public static final double ROBOT_MASS = (105.8) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprak max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16.0);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


    // TODO: Update after testing auto
    public static final class AutonConstants {
        public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(0.0, 0.0, 0.0);
        public static final PIDConstants AUTO_ANGLE_PID = new PIDConstants(0.0, 0.0, 0.0);
    }


    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }


  public static final class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6.0;
  }


    // TODO: Update with motor constants such as MAX_SPEED, PID_CONSTANTS, ECT.
    public static final class AlgaeProcessorConstants {
        public static final double ALGAE_PROCESSOR_DEFAULT_ANGLE = 0.0; //TODO: placeholder
        public static final double ALGAE_PROCESSOR_MAX_SPEED = 0.0; // TODO: placeholder
        public static final double ALGAE_PROCESSOR_MIN_SPEED = 0.0; //TODO: placeHolder
        public static final double ALGAE_PROCESSOR_P = 0.1; //TODO: tune pid values
        public static final double ALGAE_PROCESSOR_I = 0.0;
        public static final double ALGAE_PROCESSOR_D = 0.0;
        public static final double ALGAE_PROCESSOR_MAX_ANGLE = 0.0; // TODO: placeholder
        public static final double ALGAE_PROCESSOR_MIN_ANGLE = 0.0; // TODO: placeholder
        public static final double ALGAE_PROCESSOR_DEPLOY_ANGLE = 0.0; //TODO: placeholder

        public static final double ALGAE_PROCESSOR_WHEEL_MAX_SPEED = 1.0; // TODO: placeholder
        public static final double ALGAE_PROCESSOR_INTAKE_SPEED = 0.0;
        public static final double ALGAE_PROCESSOR_SWING_SPEED = 0.0;
    }


  public static final class ClimberConstants {
    //Climb voltage requires much higher values than move voltage.
    public static final int CLIMBER_CLIMB_VOLTAGE = 0; //TODO: placeHolder
    public static final int CLIMBER_MOVE_VOLTAGE = 0; //TODO: placeHolder
    public static final int TFChannel = 1;
    public static final double PROPORTIONAL = 0.05;
    public static final double INTEGRAL = 0.0;
    public static final double DERIVATIVE = 0.0;
    // Subject to change, ask Chris later. 25:1 but it isn't linear.
    public static final double GEARRATIO = 0.04;
    // Subject to change, change later after testing.
    public static final double TARGETANGLE = 110.0;
    public static final double MAXSPEED = 0.1;
    public static final double MINSPEED = 0.05;
  }


  public static final class DeAlgaeConstants {
    public static final double DE_ALGAE_DEFAULT_ANGLE = 15.0; //TODO: find arm offset
    public static final double DE_ALGAE_FLAT_ANGLE = 185.0; //TODO: find the 'distance' of 90 degrees
    public static final double DE_ALGAE_MAX_SPEED = 0.25; // TODO: placeholder
    public static final double DE_ALGAE_MIN_SPEED = 0.08; //TODO: placeHolder
    public static final double DE_ALGAE_P = 0.1; //TODO: tune pid values
    public static final double DE_ALGAE_I = 0.0;
    public static final double DE_ALGAE_D = 0.0;
    public static final double DE_ALGAE_MAX_ANGLE = 150.0; // TODO: find max arc
    public static final double DE_ALGAE_MIN_ANGLE = 75.0; // TODO: find min arc

    public static final double DE_ALGAE_WHEEL_MAX_SPEED = 1.0; // TODO: placeholder
  }


    public static final class ElevatorConstants {
        public static final int ELEVATOR_BOTTOM_LIMIT_ID = 1;
        public static final int ELEVATOR_TOP_LIMIT_ID = 2;
        public static final int ELEVATOR_ENCODER_ID = 3;

        public static final double ELEVATOR_RAMP_RATE = 5;
        public static final double ELEVATOR_MAX_VELOCITY = 1.0;
        public static final double ELEVATOR_MAX_ACCELERATION = 3.0;

        public static final double ELEVATOR_MIN_HEIGHT = 0.0; //TODO: Update heights
        public static final double ELEVATOR_LEVEL_ONE = 0.5; // why even consider the tray? can we score with the elevator?
        public static final double ELEVATOR_LEVEL_TWO = 0.70; //70 cm
        public static final double ELEVATOR_LEVEL_THREE = 1.18; //118 cm
        public static final double ELEVATOR_LEVEL_FOUR = 1.89; //189 cm
        public static final double ELEVATOR_MAX_HEIGHT = 1.89; // TODO: Determine threshold if different from highest level

        public static final double ELEVATOR_DEFAULT_FREE_MOVE_SPEED = 0.4;
        public static final double ELEVATOR_DEFAULT_FREE_MOVE_DOWN_SPEED = 0.1;
        public static final double ELEVATOR_DEFAULT_SETPOINT_THRESHOLD = 2.5;
        public static final double ENCODER_CONVERSION_FACTOR = 2.0; // CHANGE THIS!?!?!?!?! This is the value of distance/pulses

        public static final double ELEVATOR_GEARING = 12.0; // TODO: Update with gear ratio
        public static final double ELEVATOR_CARRIAGE_MASS = 4.0; // end effector mass, with / without
        public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(1.0);
        // TODO: Update values
        public static final double ELEVATOR_P = 2.0;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;

        public static final double ELEVATOR_KS = 0.00; // Static gain in volts
        public static final double ELEVATOR_KG = 2.28; // Gravity gain in volts
        public static final double ELEVATOR_KA = 0.41; // Velocity gain in V/(m/s)
        public static final double ELEVATOR_KV = 3.07; // Acceleration gain in V/(m/s^2)
        public static final double ELEVATOR_DEFAULT_TOLERANCE = Units.inchesToMeters(1);
    }


    public static final class ElevatorYAGSLConstants {

        public static final double ELEVATOR_P = 5;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_MAX_VELOCITY = 4.0;
        public static final double ELEVATOR_MAX_ACCELERATION = 6.0;
        public static final double ELEVATOR_S = 5; //voltage to overcome static friction
        public static final double ELEVATOR_G = 0; //voltage to overcome gravity
        public static final double ELEVATOR_V = 0; //velocity
        public static final double ELEVATOR_A = 0; //acceleration
        public static final double ELEVATOR_RAMP_RATE = 5;
        public static final double ELEVATOR_GEARING = 0;
        public static final double ELEVATOR_CARRIAGE_MASS = 0;
        public static final double ELEVATOR_DRUM_RADIUS = 0;
        public static final double ELEVATOR_MIN_HEIGHT_METERS = 5;
        public static final double ELEVATOR_MAX_HEIGHT_METERS = 0;
    }

    public static final class EndEffectorConstants {
        public static final int END_EFFECTOR_BEAM_BREAK = 5; //TODO: placeHolder

        public static final double END_EFFECTOR_EJECT_VOLTAGE = 12.0;
        public static final double END_EFFECTOR_INTAKE_VOLTAGE = 3.0;
    }

  
    public static final class VisionConstants {
        // Update to 2025
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField( AprilTagFields.k2024Crescendo );
        // Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
        public final double MAXIMUM_AMBIGUITY = 0.25;
    }
}
