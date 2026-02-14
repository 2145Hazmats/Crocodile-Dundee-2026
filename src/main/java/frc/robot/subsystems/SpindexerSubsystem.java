// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
  private final TalonFX spindexerMotor;
  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {
    spindexerMotor = new TalonFX(Constants.MotorConstants.SPINDEXER_MOTOR_ID);
  }
  public void SpinMotor(double Speeds){
    spindexerMotor.set(Speeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
