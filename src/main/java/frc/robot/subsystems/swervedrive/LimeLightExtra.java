package frc.robot.subsystems.swervedrive;
import java.util.Optional;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

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
}
