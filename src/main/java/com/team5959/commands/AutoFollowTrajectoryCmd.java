// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import java.util.List;

import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoFollowTrajectoryCmd extends Command {

 
  private final SwerveControllerCommand swerveControllerCommand;
  private final SwerveChassis swerveChassis;

   

  
  /** Creates a new AutoFollowTrajectoryCmd. */
  public AutoFollowTrajectoryCmd(SwerveChassis swerveChassis) {
    this.swerveChassis = swerveChassis;
   addRequirements(swerveChassis);

    // 1. Create trajectory settings
   TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    SwerveConstants.MAX_AUTO_SPEED,
    SwerveConstants.MAX_AUTO_ACCELERATION)
            .setKinematics(SwerveConstants.DRIVE_KINEMATICS);

// 2. Generate trajectory
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
            new Translation2d(1, 2),
            new Translation2d(2, 0)),
    new Pose2d(3, 3, Rotation2d.fromDegrees(180)),
    trajectoryConfig);

// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(SwerveConstants.KP_AUTO_XController, 0, 0);
PIDController yController = new PIDController(SwerveConstants.KP_AUTO_YController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
    SwerveConstants.KP_AUTO_ROTATION, 0, 0, SwerveConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
// Removed from here and moved into the constructor

// 4. Construct command to follow trajectory
swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveChassis::getPose,
        SwerveConstants.DRIVE_KINEMATICS,
        xController,
        yController,
        thetaController,
        swerveChassis::setModuleStates,
        swerveChassis);

        swerveChassis.resetNavx();
        swerveChassis.resetOdometry(trajectory.getInitialPose());
         swerveChassis.publishTrajectory("trajectory", trajectory);


  
    // Use addRequirements() here to declare subsystem dependencies.

    // Enable continuous input for thetaController
   

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
    swerveControllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveControllerCommand.execute();

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveControllerCommand.end(interrupted);
    swerveChassis.resetHeadingHoldAfterGyroReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveControllerCommand.isFinished();
  }
}




      



        

