// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swervedrive.drivebase;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.RawFiducial;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AutoMoveToTag extends Command {
//   private final static TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//   private final static TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//   private final static TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

//   //This was the tag number in the video. Change later
//   private static final int TAG_TO_CHASE = 2;
//   private static final Transform3d TAG_TO_GOAL = 
//       new Transform3d(
//         //This means to go 1.5 units directly infront of the april tag and rotated by 180 degrees
//         new Translation3d(1.5,0.0,0.0),
//         new Rotation3d(0.0,0.0, Math.PI));

//   //They added the line private final PhotonCamer photonCamera but we use limelight so change somehow
//   //These next to lines they wrtoe specific to them
//   private final SwerveSubsystem drivetrainSubsystem;
//   private final Supplier<Pose2d> poseProvider;

//   //In the place of a, b, and c put PIDS
//   private final ProfiledPIDController xController = new ProfiledPIDController(a,b,c, X_CONSTRAINTS);
//   private final ProfiledPIDController yController = new ProfiledPIDController(a,b ,c, Y_CONSTRAINTS);
//   private final ProfiledPIDController omegaController = new ProfiledPIDController(a,b ,c, OMEGA_CONSTRAINTS);

//   //They added this line for using Photon
//   private Integer lastTarget;

//   private final String Limelight_ID;
//   //private final 

//   //they are using their specs for this
//   public AutoMoveToTag(
//         String Limelight_ID,
//         SwerveSubsystem drivetrainSubsystem,
//         Supplier<Pose2d> poseProvider){
//     this.Limelight_ID = Limelight_ID;
//     this.drivetrainSubsystem = drivetrainSubsystem;
//     this.poseProvider = poseProvider;
    
//     //these are all their specs
//     xController.setTolerance(0.2);
//     yController.setTolerance(0.2);
//     omegaController.setTolerance(Units.degreesToRadians(3));
//     omegaController.enableContinuousInput(-Math.PI, Math.PI);

//     addRequirements(drivetrainSubsystem);
  
//   }
//   //they didnt have this class

//   /** Creates a new AutoMoveToTag. */
//   //public AutoMoveToTag() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   //}

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // lastTarget = null;
//     var robotPose = poseProvider.get();
//     omegaController.reset(robotPose.getRotation().getRadians());
//     xController.reset(robotPose.getX());
//     yController.reset(robotPose.getY());

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     var robotPose2d = poseProvider.get();
//     var robotPose = 
//         new Pose3d(
//             robotPose2d.getX(),
//             robotPose2d.getY(),
//             0.0,
//             new Rotation3d(0.0,0.0,robotPose2d.getRotation().getRadians())
//             //Ian Borden doesn't know how to end the line above as the video didn't show the full line
//             );
//     //find out what getlatestresult is 
//     RawFiducial[] limeLightOutput = LimelightHelpers.getRawFiducials(Limelight_ID);
//     if (limeLightOutput.length != 0){
//       // var targetOpt = photonRes.getTargets().stream()
//       //     .filter(t -> t.getFiducialId()==TAG_TO_CHASE)
//       //     .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <=0.2 && t.getPose)
//       //     //the line above is incomplete as the full line was not shown in the video
//       //     .findFirst();
//       int target;
//       boolean isPresent = false;
//       for (RawFiducial x: limeLightOutput) {
//         if (x.id == TAG_TO_CHASE && x.ambiguity <= 0.2 && x.id != lastTarget) {
//           isPresent = true;
//           break;
//         }
//       }
//       if(isPresent){
//         LimelightHelpers.SetFiducialIDFiltersOverride(Limelight_ID, new int[]{TAG_TO_CHASE}); // Only track these tag IDs
//         double[] currentPos = LimelightHelpers.getTargetPose_RobotSpace(Limelight_ID);
//         LimelightHelpers.SetFiducialIDFiltersOverride(Limelight_ID, new int[]{}); // Only track these tag IDs
//         Pose3d poseToMove = new Pose3d(currentPos[0], currentPos[1], currentPos[2], new Rotation3d(currentPos[3], currentPos[4], currentPos[5]));
//         // var target = targetOpt.get();
//         // lastTarget = target;

//         // var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

//         // var camToTarget = target.getBestCameraToTarget();
        
//         // var targetPose = cameraPose.transformBy(camToTarget);

//         Pose2d goalPose = poseToMove.transformBy(TAG_TO_GOAL).toPose2d();

//         xController.setGoal(goalPose.getX());
//         yController.setGoal(goalPose.getY());
//         omegaController.setGoal(goalPose.getRotation().getRadians());
//       }
      

//     }

//     if (lastTarget == null){
//       // drivetrainSubsystem.stop();
//     } else{
//       var xSpeed = xController.calculate(robotPose.getX());
//       if(xController.atGoal()){
//         xSpeed = 0;
//       }
//       var ySpeed = yController.calculate(robotPose.getY());
//       if (yController.atGoal()){
//         ySpeed = 0;
//       }
//       var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//       if (omegaController.atGoal()){
//         omegaSpeed = 0;
//       }

//       drivetrainSubsystem.drive(
//         ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,omegaSpeed, robotPose2d.get(/*get something*/)))
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivetrainSubsystem.stop;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
