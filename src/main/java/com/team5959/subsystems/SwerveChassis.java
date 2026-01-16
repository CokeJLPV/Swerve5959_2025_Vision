package com.team5959.subsystems;

import java.util.List;

//Vision
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.team5959.Constants.SwerveConstants;
import com.team5959.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveChassis extends SubsystemBase{

  private final PIDController headingPID = new PIDController(SwerveConstants.KP_AUTO_HOLDING,SwerveConstants.KI_AUTO_HOLDING, SwerveConstants.KD_AUTO_HOLDING); // TUNEAR

    private double headingSetpointDeg = 0.0;
    private boolean headingHoldEnabled = false;

 /* * * INITIALIZATION * * */
 
  //initialize SwerveModules 
  private SwerveModule[] swerveModules; 

  //odometer 
  private SwerveDriveOdometry odometer; 
  private SwerveDrivePoseEstimator poseEstimator;
  private AHRS navx; 

  private Vision vision;

  Field2d field2d = new Field2d();  

  public SwerveChassis(Vision vision) {

    this.vision = vision;

    headingPID.enableContinuousInput(-180.0, 180.0);
    headingPID.setTolerance(SwerveConstants.HOLDING_TOLLERANCE); // TUNEAR
    headingPID.setIZone(5);

    SmartDashboard.putData("Field", field2d);

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), //Front Left Module
      new SwerveModule(1, SwerveConstants.BackLeft.constants), //Back Left Module
      new SwerveModule(2, SwerveConstants.FrontRight.constants), //Front Right Module
      new SwerveModule(3, SwerveConstants.BackRight.constants)//Back Right Module
    };

    //instantiate navx 
    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    navx.setAngleAdjustment(0); //adjustment may be needed depending on robot orientation
    //reset navx after 1 second to ensure proper initialization
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetNavx();
      } catch (Exception e) {
        
      }
    }).start();

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRotation2d(), 
      getModulePositions(),
      new Pose2d(0, 0, getRotation2d())
    );

    //instantiate pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRotation2d(),
      getModulePositions(),
      new Pose2d(0, 0, getRotation2d())
    );  


    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    
      AutoBuilder.configure(
        this::getPose2d, 
        this::resetOdometry, 
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(
        new PIDConstants(SwerveConstants.KP_TRANS_PATHPLANNER, SwerveConstants.KI_TRANS_PATHPLANNER, SwerveConstants.KD_TRANS_PATHPLANNER),
        new PIDConstants(SwerveConstants.KP_ROT_PATHPLANNER, SwerveConstants.KI_ROT_PATHPLANNER, SwerveConstants.KD_ROT_PATHPLANNER)),
         config,
         () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this);
  }

  public void holdCurrentHeading() {
    headingSetpointDeg = getRotation2d().getDegrees();
    headingHoldEnabled = true;
}

public void disableHeadingHold() {
    
    headingHoldEnabled = false;
}

/** Llamar cuando se resetea el gyro/navX */
public void resetHeadingHoldAfterGyroReset() {
  headingSetpointDeg = getRotation2d().getDegrees(); // normalmente 0 tras reset
  headingPID.reset();
  headingHoldEnabled = false; // o true, según lo que quieras
}

   //Methods

  public void resetNavx() {
    navx.reset();

  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public Pose2d getPose2d(){
    return poseEstimator.getEstimatedPosition();
  }
/* 
   public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }  */

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    return new ChassisSpeeds(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond
    );
  }

  public void driveRobotRelative(ChassisSpeeds chassis) {
    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    setModuleStates(state);
  }

  /* * * STATES * * */

  //SET STATES 
  //gets a SwerveModuleStates array from driver control and sets each module to the corresponding SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states; 
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
  }


  

  //LOCK 
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));//Front left
    states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));//BackLeft
    states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));//Front Right
    states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));//BackRight

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }
  }

  //STRAIGHTEN THE WHEELS 
  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }
  }

  //DRIVE
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
    SwerveModuleState[] states;
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d())

      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    setModuleStates(states);

  }

  private boolean wasRotating = false;  // true si en el ciclo anterior |zSpeed| >= deadband

  public void driveWithHeadingHold(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
      double omega;
  
      boolean rotatingNow = Math.abs(zSpeed) >= 0.05;
  
      if (!rotatingNow) { // deadband Z → mantener heading
          // flanco de bajada: veníamos girando y ahora NO
          if (wasRotating || !headingHoldEnabled) {
              // Captura el heading EXACTO en el primer ciclo de reposo
              headingSetpointDeg = getRotation2d().getDegrees();
              headingPID.reset();
              headingHoldEnabled = true;
          }
  
          double currentHeadingDeg = getRotation2d().getDegrees();
          double pidOutput = headingPID.calculate(currentHeadingDeg, headingSetpointDeg);
          pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
          omega = pidOutput;
      } else {
          // joystick Z manda
          disableHeadingHold();
          omega = zSpeed;
      }
  
      wasRotating = rotatingNow;
  
      drive(xSpeed, ySpeed, omega, fieldOriented);
  }

  //STOP 
  public void stopModules() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }
}

public void resetDriveEncoders() {
  for (SwerveModule swerveMod : swerveModules) {
    swerveMod.resetDriveEncoder();
  }
}

public void publishTrajectory(String name, Trajectory trajectory) {
  field2d.getObject(name).setTrajectory(trajectory);
}

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());
    poseEstimator.update(getRotation2d(), getModulePositions());

    if (Math.abs(getRobotRelativeSpeeds().omegaRadiansPerSecond) > 3.5) {
      // Actualizamos solo el field2d y salimos
      field2d.setRobotPose(odometer.getPoseMeters());
      return; 
    }

    List<EstimatedRobotPose> visionEstimates = vision.getEstimatedGlobalPoses(poseEstimator.getEstimatedPosition());

    for (EstimatedRobotPose estimate : visionEstimates){
      //Calculamos la confianza dinámica antes de agregar la medición
      var stdDevs = vision.getEstimationStdDevs(estimate);

      poseEstimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs);
    }

    field2d.setRobotPose(odometer.getPoseMeters());

    SmartDashboard.putData("NAVX2D", navx);
    SmartDashboard.putString("POSE INFO", odometer.getPoseMeters().toString());
    
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }
   
    //SmartDashboard.putNumber("NAVX", -navx.getAngle());
    //SmartDashboard.putNumber("NAVXYAW", navx.getYaw());
    //SmartDashboard.putNumber("rot 2d", ((getRotation2d().getDegrees() % 360) + 360) % 360);

    SmartDashboard.putNumber("Distancia FL", swerveModules [0].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RL", swerveModules [1].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia FR", swerveModules [2].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RR", swerveModules [3].getPosition().distanceMeters);

    SmartDashboard.putNumber("Speed FL", swerveModules [0].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Speed RL", swerveModules [1].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Speed FR", swerveModules [2].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Speed RR", swerveModules [3].getState().speedMetersPerSecond);


    

    //SmartDashboard.putNumber("HEADING SP", headingSetpointDeg);
  }

  /* * * ADDED METHODS * * */
public double deadzone(double num){
  return Math.abs(num) > 0.1 ? num : 0;
}

@SuppressWarnings("unused")
private static double modifyAxis(double num) {
// Square the axis
num = Math.copySign(num * num, num);

return num;
}
/*
 * Este método corre automáticamente solo cuando estás simulando el robot.
*/
@Override
public void simulationPeriodic() {
    // Le decimos a PhotonVision dónde está el robot simulado en el campo.
    // Esto permite que las cámaras simuladas "vean" los AprilTags desde esa posición.
    // NOTA: Normalmente aquí se usaria 'poseEstimator.getEstimatedPosition()', 
    // pero en simulación pura, a veces es mejor usar la pose física ideal si se tiene PhysicsSim.
    // Por ahora, usar la odometría es suficiente para pruebas básicas.
    vision.simulationPeriodic(poseEstimator.getEstimatedPosition());
}

}
