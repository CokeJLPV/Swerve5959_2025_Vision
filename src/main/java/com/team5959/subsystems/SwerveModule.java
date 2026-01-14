package com.team5959.subsystems;
import com.revrobotics.spark.SparkMax; // Clase para controlar motores SparkMax
import com.revrobotics.spark.SparkBase.PersistMode; // Modo de persistencia de configuración
import com.revrobotics.spark.SparkBase.ResetMode; // Modo de reinicio de configuración
import com.revrobotics.spark.config.SparkMaxConfig; // Configuración para SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // Modos de inactividad del motor
import com.revrobotics.spark.SparkLowLevel.MotorType; // Tipos de motor (Brushless o Brushed)

import edu.wpi.first.math.controller.PIDController; // Controlador PID para control de motores

import edu.wpi.first.math.geometry.Rotation2d; // Representación de rotación en 2D

import edu.wpi.first.math.kinematics.SwerveModulePosition; // Posición de un módulo swerve
import edu.wpi.first.math.kinematics.SwerveModuleState; // Estado de un módulo swerve

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Herramienta para mostrar datos en tiempo real

import com.team5959.SwerveModuleConstants; // Constantes específicas para el módulo swerve
import com.team5959.Constants.SwerveConstants; // Constantes generales para el sistema swerve

import com.revrobotics.RelativeEncoder; // Clase para manejar encoders relativos

import com.ctre.phoenix6.configs.MagnetSensorConfigs; // Configuración para sensores magnéticos
import com.ctre.phoenix6.signals.SensorDirectionValue; // Valores de dirección del sensor
import com.ctre.phoenix6.hardware.CANcoder; // Clase para manejar encoders absolutos CANcoder

public class SwerveModule {
    /* * * INITIALIZATION * * */

    SparkMaxConfig driveConfig;
    SparkMaxConfig rotationConfig;

    public int moduleID; 
    //initialize motors 
    private SparkMax driveMotor; 
    private SparkMax rotationMotor; 

    //initialize encoders 
    private CANcoder absoluteEncoder; 
    private RelativeEncoder driveEncoder; 

    //init PID Controller for turning 
    private PIDController rotationPID; 

    //init info 
    private MagnetSensorConfigs absoluteEncoderConfigs;
    private double encOffset; 
    private boolean encInverted;

    /* * * CONSTRUCTOR * * */
    /* 
     * @param moduleID the id of the module 
     * @param moduleConstants a SwerveModuleConstants obj 
     */

    public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID; //used to differentiate between the four swerve modules in the SwerveSubsystem class 
        encOffset = moduleConstants.angleOffset;
        encInverted = moduleConstants.canCoderInverted;

        //instantiate drive motor and encoder 
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
        driveConfig = new SparkMaxConfig();
        driveEncoder = driveMotor.getEncoder();

        //instantiate rotation motor and absolute encoder 
        rotationMotor = new SparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        rotationConfig = new SparkMaxConfig();
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);

        /* * * DRIVE MOTOR * * */
        //CONFIGURATIONS
        
        driveConfig.inverted(moduleConstants.driveInverted);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(35);
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION);
       

        driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        /* * * ROTATION MOTOR * * */
        //CONFIGURATIONS
        rotationConfig.inverted(moduleConstants.rotationInverted);
        rotationConfig.idleMode(IdleMode.kBrake);
        rotationConfig.smartCurrentLimit(25);

        rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        absoluteEncoderConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(-encOffset);

        absoluteEncoder.getConfigurator()
        .apply(absoluteEncoderConfigs);


        //configure rotation PID controller 
        rotationPID = new PIDController(
            SwerveConstants.KP_TURNING, 
            SwerveConstants.KI_TURNING, 
            SwerveConstants.KD_TURNING);
        rotationPID.enableContinuousInput(-180, 180); //Continuous input considers min & max to be the same point; calculates the shortest route to the setpoint 
        rotationPID.setTolerance(1); //1.5 degrees of tolerance

        resetDriveEncoder();
    } 

    /* * * GET METHODS * * */

    private double drivePosition() {
        return driveEncoder.getPosition();
    }

    private double getAbsoluteEncoderDegrees() {
        return ((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360))*(encInverted? -1.0 : 1.0);
    }

    private double driveVelocity() {
        return driveEncoder.getVelocity();
    }

   /*  private double rotationVelocity (){
        return absoluteEncoder.getVelocity().getValueAsDouble();
    } */

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);        
    }

   

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    //returns a new SwerveModulePosition representing the current drive position and rotation motor angle 
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    /* * * SET METHODS * * */

    public void setState(SwerveModuleState desiredState) {
        //optimize state so the rotation motor doesnt have to spin as much 
        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        Rotation2d currentAngle = getState().angle;

        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);    
        
        desiredState.optimize(currentAngle);
        SwerveModuleState optimizedState = desiredState;

        double rotationOutput = rotationPID.calculate(currentAngle.getDegrees(), optimizedState.angle.getDegrees());

        rotationOutput = Math.max(-1, Math.min(1, rotationOutput));

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED); 

        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DESIRED ANG DEG", optimizedState.angle.getDegrees());
        SmartDashboard.putString("Swerve Module " + moduleID + " State", optimizedState.toString());
    }

    public void setAngle(SwerveModuleState desiredState) {
        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        Rotation2d currentAngle = getState().angle;
        
        desiredState.optimize(currentAngle);
        SwerveModuleState targetAngle = desiredState;

        double rotationOutput = rotationPID.calculate(currentAngle.getDegrees(), targetAngle.angle.getDegrees());

        rotationOutput = Math.max(-1, Math.min(1, rotationOutput));

        rotationMotor.set(rotationOutput); 
        driveMotor.set(0);

       
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

   

   

    public void print() {
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] DRIVE SPEED", driveVelocity());
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] ROTATION SPEED", absoluteEncoder.getVelocity().getValueAsDouble());
        SmartDashboard.putString("S["+absoluteEncoder.getDeviceID()+"] CURRENT STATE", getState().toString());

        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DRIVE MOTOR OUTPUT", driveMotor.get());
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ROTATION MOTOR OUTPUT", rotationMotor.get());

        
        
        

        
    }
}

