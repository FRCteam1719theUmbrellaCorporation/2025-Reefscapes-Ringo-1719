// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoMoveToTag extends Command {
  private final static TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final static TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private final static TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  //This was the tag number in the video. Change later
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
        //This means to go 1.5 units directly infront of the april tag and rotated by 180 degrees
        new Translation3d(1.5,0.0,0.0),
        new Rotation3d(0.0,0.0, Math.PI));

  //They added the line private final PhotonCamer photonCamera but we use limelight so change somehow
  //These next to lines they wrtoe specific to them
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;

  //In the place of a, b, and c put PIDS
  private final ProfiledPIDController xController = new ProfiledPIDController(a,b ,c, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(a,b ,c, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(a,b ,c, OMEGA_CONSTRAINTS);

  //They added this line for using Photon
  private PhotonTrackedTarget lastTarget;

  //they are using their specs for this
  public ChaseTagCommand(
        PhotonCamera photonCamera,
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider){
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    
    //these are all their specs
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  
  }

  /** Creates a new AutoMoveToTag. */
  public AutoMoveToTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
