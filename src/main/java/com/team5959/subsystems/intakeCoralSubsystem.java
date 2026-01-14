// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team5959.Constants.intakeCoralConstants;

public class intakeCoralSubsystem extends SubsystemBase {

  // INIZIALIATION

  // initialize motors
  private final SparkMax coralIntakeMotorRight;
  private final SparkMax coralIntakeMotorLeft;

  // initialize motor configuration
  private final SparkBaseConfig coralIntakeMotorRightConfig;
  private final SparkBaseConfig coralIntakeMotorLeftConfig;

  public intakeCoralSubsystem() {
    // instatiate motors and config
    coralIntakeMotorRight = new SparkMax(intakeCoralConstants.coralIntakeMotorRightID, MotorType.kBrushless);
    coralIntakeMotorLeft = new SparkMax(intakeCoralConstants.coralIntakeMotorLeftID, MotorType.kBrushless);

    coralIntakeMotorRightConfig = new SparkMaxConfig();
    coralIntakeMotorLeftConfig = new SparkMaxConfig();

    coralIntakeMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    coralIntakeMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    coralIntakeMotorLeftConfig.follow(coralIntakeMotorRight, intakeCoralConstants.coralIntakeMotorLeftInverted);
    coralIntakeMotorRightConfig.inverted(intakeCoralConstants.coralIntakeMotorRightInverted);

    coralIntakeMotorRight.configure(coralIntakeMotorRightConfig, null, null);
    coralIntakeMotorLeft.configure(coralIntakeMotorLeftConfig, null, null);
  }

  public void runInCoralIntake() {
    coralIntakeMotorRight.set(0.7);
  }

  public void runOutCoralIntake() {
    coralIntakeMotorRight.set(-0.7);
  }

  public void stopCoralIntake() {
    coralIntakeMotorRight.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
