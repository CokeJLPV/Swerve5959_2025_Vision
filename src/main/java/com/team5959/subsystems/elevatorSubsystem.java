// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

//Sensores CAN CTRE
import com.ctre.phoenix6.CANBus; //Sensor de rango y proximidad CANrange
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team5959.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevatorSubsystem extends SubsystemBase {

  // INITIALIZATION

  // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange canRange = new CANrange(30, kCANBus);

  // initialize motors
  private final SparkMax elevatorRight;
  private final SparkMax elevatorLeft;

  // initialize motor configuration
  private final SparkBaseConfig elevatorLeftConfig;
  private final SparkBaseConfig elevatorRightConfig;

  // initialize encoder
  private final RelativeEncoder elevatorEncoder;

  // initialize PID controller
  private final PIDController elevatorPID;
  private final PIDController elevatorStartingPositionPID;

  // Target position
  private double targetPosition;

  // LIMITS
  // Soft Limit Configuration
  SoftLimitConfig elevatorRightSoftLimitConfig;
  SoftLimitConfig elevatorLeftSoftLimitConfig;

  public elevatorSubsystem() {

    // Configuracion de sensor CanRange
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000 its valid.
    config.ProximityParams.ProximityThreshold = 0.1; // If CANrange detects an object within 0.2 meters, it will trigger
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at
    canRange.getConfigurator().apply(config);// Apply the configuration to the CANrange

    // instatiate motors, config and encoder
    elevatorRight = new SparkMax(ElevatorConstants.elevatorRightID, MotorType.kBrushless);
    elevatorLeft = new SparkMax(ElevatorConstants.elevatorLeftID, MotorType.kBrushless);

    elevatorEncoder = elevatorRight.getEncoder();

    elevatorEncoder.setPosition(0);

    elevatorPID = new PIDController(ElevatorConstants.KP_ELEVATOR, ElevatorConstants.KI_ELEVATOR,ElevatorConstants.KD_ELEVATOR);
    elevatorStartingPositionPID = new PIDController(ElevatorConstants.KP_SP_ELEVATOR, ElevatorConstants.KI_ELEVATOR,ElevatorConstants.KP_ELEVATOR);

    elevatorRightConfig = new SparkMaxConfig();
    elevatorLeftConfig = new SparkMaxConfig();
    elevatorRightSoftLimitConfig = new SoftLimitConfig();
    elevatorLeftSoftLimitConfig = new SoftLimitConfig();

    elevatorRightSoftLimitConfig.forwardSoftLimit(ElevatorConstants.elevatorLowerLimit); // positive values go down
    elevatorRightSoftLimitConfig.reverseSoftLimit(ElevatorConstants.elevatorUpperLimit); // negative values go up
    elevatorRightSoftLimitConfig.forwardSoftLimitEnabled(ElevatorConstants.forwardSoftLimitEnabled);
    elevatorRightSoftLimitConfig.reverseSoftLimitEnabled(ElevatorConstants.reverseSoftLimitEnabled);

    elevatorRightConfig.apply(elevatorRightSoftLimitConfig);

    elevatorLeftConfig.follow(elevatorRight, ElevatorConstants.elevatorLeftInverted);
    elevatorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    elevatorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    elevatorRightConfig.inverted(ElevatorConstants.elevatorRightInverted);

    elevatorLeft.configure(elevatorLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorRight.configure(elevatorRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setMotorSpeed(double speed) {
    // If the CANrange detects an object within the proximity threshold, stop the elevator
    if (canRange.getIsDetected().getValue() == false) {
      elevatorRight.set(0);
    } else {
      elevatorRight.set(speed);
    }
  }

  public boolean canRangeDetectsObject() {
    return canRange.getIsDetected().getValue();
  }

  public void CurrentToTargetPosition() {
    // Set target to current position
    targetPosition = elevatorEncoder.getPosition();
  }

  // PRESET POSITIONS
  public void moveToStartingPosition() {
    // Move to preset position 0
    setTargetPosition(ElevatorConstants.elevatorStartingPosition);
  }

  public void moveToL1Position() {
    // Move to preset L1 position
    setTargetPosition(ElevatorConstants.elevatorL1Position);
  }

  public void moveToL2Position() {
    // Move to preset L2 position
    setTargetPosition(ElevatorConstants.elevatorL2Position);
  }

  public void moveToL3Position() {
    // Move to preset L3 position
    setTargetPosition(ElevatorConstants.elevatorL3Position);
  }

  // Method to set a target position
  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  public double getCurrentPosition() {
    return elevatorEncoder.getPosition();
  }

  public void elevatorUpManualMode() {
    setMotorSpeed(0.4);
  }

  public void elevatorDownManualMode() {
    setMotorSpeed(-0.4);
  }

  public void stopElevator() {
    elevatorRight.set(0);
  }

  // Method to check if the motor has reached the target position
  public boolean atTargetPosition() {
    return Math.abs(getCurrentPosition() - targetPosition) <= 2;
  }

  public void runPIDElevatorTarget(){
    double pidOutput;
    if (targetPosition == ElevatorConstants.elevatorStartingPosition){
      pidOutput = elevatorStartingPositionPID.calculate(elevatorEncoder.getPosition(), targetPosition);
    } else {
      pidOutput = elevatorPID.calculate(elevatorEncoder.getPosition(), targetPosition);
    }
    pidOutput = MathUtil.clamp(pidOutput, -0.7, 0.7); // Clamp output to safe range
    setMotorSpeed(pidOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("CanRange Coral", canRange.getIsDetected().getValue());

  }
}
