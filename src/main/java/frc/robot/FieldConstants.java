package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/* File contains field locations for vision targets and other field elements
 * Concept taken from FRC Team 6328, Mechanical Advantage
 * https://github.com/Mechanical-Advantage/RobotCode2023/blob/245956d9635309737d78d7f915cfd6d1b94167a1/src/main/java/org/littletonrobotics/frc2023/FieldConstants.java#L194
 */
public final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(313.25);

    public static final AprilTagFieldLayout aprilTagMap = new AprilTagFieldLayout(getAprilTags(), fieldLength,
            fieldWidth);

    // Static method to generate list of AprilTags
    public static List<AprilTag> getAprilTags() {
        List<AprilTag> aprilTags = new ArrayList<AprilTag>();

        aprilTags.add(
                new AprilTag(1,
                        new Pose3d(
                            Units.inchesToMeters(593.68),
                            Units.inchesToMeters(9.68),
                            Units.inchesToMeters(53.38),
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(120)))));

        aprilTags.add(
                new AprilTag(2,
                        new Pose3d(
                            Units.inchesToMeters(637.21),
                            Units.inchesToMeters(34.79),
                            Units.inchesToMeters(53.38),
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(120)))));
        
        aprilTags.add(
                new AprilTag(3,
                        new Pose3d(
                            Units.inchesToMeters(652.73),
                            Units.inchesToMeters(196.17),
                            Units.inchesToMeters(57.13),
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)))));

        aprilTags.add(
                new AprilTag(4,
                        new Pose3d(
                            Units.inchesToMeters(652.73),
                            Units.inchesToMeters(218.42),
                            Units.inchesToMeters(57.13),
                            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)))));

        return aprilTags;
    }

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with
     * the origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent() & ally.get() == Alliance.Red) {
            // If driverstation is present and we are on the RED alliance, flip the field
            // orientation
            return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
            // Default origin for the field is the BLUE alliance wall
            return translation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance
     * color. By default,
     * all translations and poses in {@link FieldConstants} are stored with the
     * origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Pose2d allianceFlip(Pose2d pose) {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent() & ally.get() == Alliance.Red) {
            // If driverstation is present and we are on the RED alliance, flip the pose
            // orientation
            return new Pose2d(
                    fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            // Default origin for the field is the BLUE alliance wall
            return pose;
        }
    }
}
