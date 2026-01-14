package com.team5959.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.team5959.Constants.intakeAlgaeConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armIntakeAlgaeSubsystem extends SubsystemBase {
    //INITIALIZATION

    //initialize motors
    private final SparkMax armMotor;
    private final SparkMax algaeIntakeMotor;

    //initialize motor configuration
    private SparkMaxConfig algaeIntakeMotorConfig;

    //initialize PID controller
    private final PIDController armPID;

    //Target position
    double armTargetPosition = 70;

    //Encoder Absolute Position
    DutyCycleEncoder armAbsoluteEncoder;
    RelativeEncoder positionSpark;

    //Switch State
    private boolean isArmOut;

    public armIntakeAlgaeSubsystem() {
        algaeIntakeMotor = new SparkMax(intakeAlgaeConstants.algaeIntakeMotorID, MotorType.kBrushless);
        armMotor = new SparkMax(intakeAlgaeConstants.armMotorID, MotorType.kBrushless);

        algaeIntakeMotorConfig = new SparkMaxConfig();

        algaeIntakeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        algaeIntakeMotorConfig.inverted(intakeAlgaeConstants.algaeIntakeMotorInverted);

        armPID = new PIDController(intakeAlgaeConstants.KP_ARM, intakeAlgaeConstants.KI_ARM, intakeAlgaeConstants.KD_ARM);

        armAbsoluteEncoder = new DutyCycleEncoder(intakeAlgaeConstants.absoluteEncoderPort);

        positionSpark = armMotor.getEncoder();
    }

    public void setAlgaeIntakeSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }

    // MÉTODO AUXILIAR para claridad en el comando
    public void stopAlgaeIntake() {
        algaeIntakeMotor.set(0.0);
    }

    public double currentArmPosition() {
        double multiplicationOne = armAbsoluteEncoder.get() * 360;
        double multiplicationTwo = (int)multiplicationOne;
        return multiplicationTwo;

    }

    public void currentToTargetPosition() {
        armTargetPosition = currentArmPosition();
    }

    public void setArmTargetPosition(double armTargetPosition) {
        this.armTargetPosition = armTargetPosition;
    }

    public void moveToInPosition() {
        setArmTargetPosition(intakeAlgaeConstants.armIntakeInStartingPosition);
    }

    public void moveToOutPosition() {
        setArmTargetPosition(intakeAlgaeConstants.armIntakeOutPosition);
    }

    public void inorOutPositionSwitch(){
        if (isArmOut){
            moveToInPosition();
            isArmOut = false;
        } else {
            moveToOutPosition();
            isArmOut = true;
        }
    }

    public void moveToInPerimeterPosition() {
        setArmTargetPosition(intakeAlgaeConstants.armIntakeInPerimeterPosition);
    }

    public void runPIDArmTarget() {
        double pidOutput = armPID.calculate(currentArmPosition(), armTargetPosition);
        armMotor.set(pidOutput);
    }

    public boolean isAtTargetPosition(){
        return Math.abs(currentArmPosition() - armTargetPosition) < 2.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", currentArmPosition());
    }
}