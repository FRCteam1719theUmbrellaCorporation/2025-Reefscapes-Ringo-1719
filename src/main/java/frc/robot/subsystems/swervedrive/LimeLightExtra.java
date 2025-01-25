package frc.robot.subsystems.swervedrive;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimeLightExtra {

    public final static String backCam = "limelight-back";
    public final static String frontCam = "limelight-front";

    public static Optional<RawFiducial> getBestTag(String limeLightName) {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(limeLightName);
        if (tags.length == 0) {
            System.out.println("x");
            return Optional.empty();
        }

        RawFiducial bestResult = tags[0];
        double amiguity = tags[0].ambiguity;
        double currentAmbiguity = 0;

        for (RawFiducial tag : tags) {
            if (tag.ambiguity < amiguity) {
                bestResult = tag;
                amiguity = tag.ambiguity;
            }
        }

        System.out.println(bestResult.id);
        return Optional.of(bestResult);
    }


    public static void updatePoseEstimation(SwerveDrive swerveDrive) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(backCam);
        boolean doRejectUpdate = false;

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            swerveDrive.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }

    }
}