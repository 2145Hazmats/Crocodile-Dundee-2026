// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeLeft;
  private TalonFX intakeRight;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeLeft = new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID);
    intakeRight = new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeMotors(double speed){
    intakeLeft.set(speed);
    intakeRight.set(speed);
  }
}
