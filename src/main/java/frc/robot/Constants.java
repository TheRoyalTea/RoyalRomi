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
    // Wheel Diameter
    public static double wheelDiameterInch = 2.75591;

    // Port for Drivetrain Wheel Motors
    public static int LEFT_MOTOR_PORT = 0;
    public static int RIGHT_MOTOR_PORT = 1;

    // Port for Drivetrain Wheel Encoders
    public static int LEFT_ENCODER_PORT_A = 4;
    public static int LEFT_ENCODER_PORT_B = 5;
    public static int RIGHT_ENCODER_PORT_A = 6;
    public static int RIGHT_ENCODER_PORT_B = 7;

    // Counts per Revolution for Encoders
    public static double countPerRevolution = 1440.0;

    // Port for Controllers
    public static int DRIVER_GAMEPAD_PORT = 0;
}
