// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Romi robot wheel motors
  private Spark leftDriveMotor;
  private Spark rightDriveMotor;

  // Encoders for wheel motors
  private Encoder leftDriveEncoder;
  private Encoder rightDriveEncoder;

  // Differential Drive for wheel motors
  private DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Romi robot wheel motors
    this.leftDriveMotor = new Spark(Constants.LEFT_MOTOR_PORT);
    this.rightDriveMotor = new Spark(Constants.RIGHT_MOTOR_PORT);

    // Romi robot encoders
    this.leftDriveEncoder = new Encoder(Constants.LEFT_ENCODER_PORT_A, Constants.LEFT_ENCODER_PORT_B);
    this.rightDriveEncoder = new Encoder(Constants.RIGHT_ENCODER_PORT_A, Constants.RIGHT_ENCODER_PORT_B);

    // Sets distance per pulse for encoders based on wheel diameter (Circumference / CPR)
    this.leftDriveEncoder.setDistancePerPulse((Math.PI * Constants.wheelDiameterInch) / Constants.countPerRevolution);
    this.rightDriveEncoder.setDistancePerPulse((Math.PI * Constants.wheelDiameterInch) / Constants.countPerRevolution);

    // Resets Encoders (Sets to Zero)
    this.leftDriveEncoder.reset();
    this.rightDriveEncoder.reset();

    // Sets up differential drive
    this.diffDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  }

  // Arcade drive for drivetrain
  public void arcadeDrive(double linear, double rotation) {
    diffDrive.arcadeDrive(linear, rotation);
  }

  // Getter for Left Wheel Distance Traveled in Inches
  public double leftWheelDistanceTraveledInch() {
    return leftDriveEncoder.getDistance();
  }

  // Getter for Right Wheel Distance Traveled in Inches
  public double rightWheelDistanceTraveledInch() {
    return rightDriveEncoder.getDistance();
  }

  // Resets encoders
  public void resetEncoders() {
    leftDriveEncoder.reset();
    rightDriveEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
