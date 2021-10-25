// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Romi robot wheel motors
  private Spark leftDriveMotor;
  private Spark rightDriveMotor;

  // Differential Drive for wheel motors
  private DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Romi robot wheel motors
    this.leftDriveMotor = new Spark(Constants.LEFT_MOTOR_PORT);
    this.rightDriveMotor = new Spark(Constants.RIGHT_MOTOR_PORT);

    // Sets up differential drive
    this.diffDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  }

  // Arcade drive for drivetrain
  public void arcadeDrive(double linear, double rotation) {
    diffDrive.arcadeDrive(linear, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
