package com.team5959.subsystems;

import com.team5959.Constants.miniArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class miniArmSubsystem extends SubsystemBase{
    //INITIALIZATION

    // initialize motors
    private final SparkMax miniArmMotor;

    // Encoder Absolute Position
    private final DutyCycleEncoder miniArmAbsoluteEncoder;
    
    // initialize PID controller
    private final PIDController miniArmPID;

    // Target position (en grados)
    private double miniArmTargetPosition;

    //Switch State
    private boolean isMiniArmOut;

    public miniArmSubsystem(){
        // instatiate motors
        miniArmMotor = new SparkMax(miniArmConstants.miniArmMotorID, MotorType.kBrushless);

        // Instatiate PID controller
        miniArmPID = new PIDController(miniArmConstants.KP_MINI_ARM, miniArmConstants.KI_MINI_ARM, miniArmConstants.KD_MINI_ARM);
        
        // Absolute Encoder
        miniArmAbsoluteEncoder = new DutyCycleEncoder(miniArmConstants.absoluteEncoderPort);
    }
    
    public void runPIDMiniArmTarget() { 
        double pidOutput = miniArmPID.calculate(currentMiniArmPosition(), miniArmTargetPosition);
        miniArmMotor.set(pidOutput);
    }

    public void stopMiniArmMotor() {
        miniArmMotor.set(0);
    }

    public double currentMiniArmPosition() {
        return (int)(miniArmAbsoluteEncoder.get() * 360);
    }

    public void currentToTargetPosition() {
      miniArmTargetPosition = currentMiniArmPosition();
    }

    public void setMiniArmTargetPosition(double miniArmTargetPosition) {
      this.miniArmTargetPosition = miniArmTargetPosition;
    }

    public void moveToStartingPosition(){
      setMiniArmTargetPosition(miniArmConstants.miniArmStartingPosition);
    }
    
    public void moveToDownPosition(){
      setMiniArmTargetPosition(miniArmConstants.miniArmDownPosition);
    }

    public void downOrStartingPositionSwitch(){
      if(isMiniArmOut){
        moveToStartingPosition();
        isMiniArmOut = false;
      } else {
        moveToDownPosition();
        isMiniArmOut = true;
      }
    }

    public void moveToDropAlgaePosition(){
      setMiniArmTargetPosition(miniArmConstants.miniArmDropAlgaePosition);
    }    
    public boolean atTargetPosition() {
      return Math.abs(currentMiniArmPosition() - miniArmTargetPosition) < 1.0;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Mini Arm Position", currentMiniArmPosition());
    }
}