// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Inner class to only hold drivetrain constants
    public final class DriveConstants {
        // Motor ports
        public static final int LEFT_MOTOR_PORT = 0;
        public static final int RIGHT_MOTOR_PORT = 1;

        // Drift voltage constant to counter right motor being slower than left
        public static final double DRIFT_VOLTAGE_CONSTANT = 0.27;

        // Wheel dimensions
        public static final double WHEEL_DIAMETER_INCHES = 2.75591; // 70 mm

        // Trackwidth of robot
        public static final double ROBOT_TRACKWIDTH_INCHES = 5.55118; // 141 mm

        // Proportional controller gains for both wheels
        public static final double LEFT_WHEEL_PROPORTIONAL_GAINS = 0.655;
        public static final double RIGHT_WHEEL_PROPORTIONAL_GAINS = 0.730;

        // Motor voltage coefficient values for voltage balance equations
        public static final double STATIC_COEFFICIENT = 0.189;
        public static final double VELOCITY_COEFFICIENT = 10.2;
        public static final double ACCELERATION_COEFFICIENT = 0.297;

        // Max velocity for drive (m/s)
        public static final double MAX_VELOCITY = 0.5;

        // Left wheel encoder ports
        public static final int LEFT_ENCODER_PORT_A = 4;
        public static final int LEFT_ENCODER_PORT_B = 5;

        // Right wheel encoder ports
        public static final int RIGHT_ENCODER_PORT_A = 6;
        public static final int RIGHT_ENCODER_PORT_B = 7;

        // Encoder resolution
        public static final double ENCODER_COUNTS_PER_REVOLUTION = 1440.0;

        // Network table name
        public static final String DRIVE_NETWORK_TABLE = "RoyalDrivetrain";
    }

    // Inner class for trajectory commands
    public final class TrajectoryConstants {
        // Trajectory controller gains (b like proportial gains and zeta like derivative gains)
        public static final double RAMSETE_B = 2.0;
        public static final double RAMSETE_ZETA = 0.8;

        // Constraints to trajectory movement
        public static final double MAX_SPEED = 0.5;
        public static final double MAX_ACCELERATION = 0.5; 

        // Name of trajectory object for field
        public static final String TRAJ_OBJECT = "traj";
    }

    // Inner class to only hold constants dealing with inputs
    public final class InputConstants {
        // Xbox controller port for driver
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
