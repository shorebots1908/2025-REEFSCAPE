package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class FieldPoint {
  public static enum pose {
    REEF_AB,
    REEF_CD,
    REEF_GH,
    REEF_IJ,
    REEF_KL,
    REEF_EF,
    CORAL_PROCESSOR1,
    CORAL_PROCESSOR2
  }

  public static List<Pose2d> scorePoses = new ArrayList<>();
  // old points
  public static final Pose2d START_1 = new Pose2d(8.2095, 7.27175, new Rotation2d());
  public static final Pose2d START_2 = new Pose2d(8.2095, 6.17, new Rotation2d());
  public static final Pose2d START_3 = new Pose2d(8.2095, 5.0584999999999996, new Rotation2d());
  public static final Pose2d REEF_GH =
      new Pose2d(5.8305, 4.03475, new Rotation2d(Units.degreesToRadians(180)));
  public static final Pose2d REEF_IJ =
      new Pose2d(5.19675, 5.17, new Rotation2d(Units.degreesToRadians(120)));
  public static final Pose2d REEF_KL =
      new Pose2d(3.83175, 5.18, new Rotation2d(Units.degreesToRadians(60)));
  public static final Pose2d REEF_AB =
      new Pose2d(3.1492500000000003, 4.015249999999999, new Rotation2d(0));
  public static final Pose2d REEF_CD =
      new Pose2d(3.83175, 2.8354999999999997, new Rotation2d(Units.degreesToRadians(300)));
  public static final Pose2d REEF_EF =
      new Pose2d(5.175, 2.855, new Rotation2d(Units.degreesToRadians(240)));
  public static final Pose2d CORAL_PROCESSOR1 =
      new Pose2d(1.56, 0.7879999999999996, new Rotation2d(Units.degreesToRadians(90)));
  public static final Pose2d CORAL_PROCESSOR2 =
      new Pose2d(0.741, 1.3339999999999999, new Rotation2d());
  public static final Pose2d CORAL_BARGE1 =
      new Pose2d(0.8482500000000001, 6.76475, new Rotation2d());
  public static final Pose2d CORAL_BARGE2 = new Pose2d(1.56, 7.27175, new Rotation2d());

  // new poses
  public static List<Pose2d> START_POSES = new ArrayList<>();
  public static List<Pose2d> LEFT_POSES = new ArrayList<>();
  public static List<Pose2d> RIGHT_POSES = new ArrayList<>();
  public static final Pose2d AB_START = new Pose2d(2.9083, 4.0259, new Rotation2d(0));
  public static final Pose2d AB_LEFT = new Pose2d(3.28295, 4.191, new Rotation2d(0));
  public static final Pose2d AB_RIGHT = new Pose2d(3.28295, 3.8608, new Rotation2d(0));
  public static final Pose2d CD_START = new Pose2d(4.073906, 3.306318, new Rotation2d(Math.PI / 3));
  public static final Pose2d CD_LEFT =
      new Pose2d(3.743600206, 3.0644211, new Rotation2d(Math.PI / 3));
  public static final Pose2d CD_RIGHT =
      new Pose2d(4.029561794, 2.8993211, new Rotation2d(Math.PI / 3));
  public static final Pose2d KL_START =
      new Pose2d(4.073906, 4.745482, new Rotation2d(5 * Math.PI / 3));
  public static final Pose2d KL_LEFT =
      new Pose2d(4.029561794, 5.1524789, new Rotation2d(5 * Math.PI / 3));
  public static final Pose2d KL_RIGHT =
      new Pose2d(3.743600206, 4.9873789, new Rotation2d(5 * Math.PI / 3));

  public static Pose2d flipPose(Pose2d pose) {
    Pose2d flippedPose;
    flippedPose =
        new Pose2d(
            (FlippingUtil.fieldSizeX - pose.getX()),
            (FlippingUtil.fieldSizeY - pose.getY()),
            pose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    return flippedPose;
  }

  public static void initScorePoses(boolean isFlipped) {

    scorePoses.add(REEF_AB);
    scorePoses.add(REEF_CD);
    scorePoses.add(REEF_EF);
    scorePoses.add(REEF_GH);
    scorePoses.add(REEF_IJ);
    scorePoses.add(REEF_KL);
    scorePoses.add(CORAL_PROCESSOR1);
    // scorePoses.add(CORAL_PROCESSOR2);
    if (isFlipped) {
      for (int i = 0; i < scorePoses.size(); i++) {
        scorePoses.set(i, flipPose(scorePoses.get(i)));
      }
    }
  }

  public static void initScorePoses() {
    initScorePoses(false);
  }

  public static void initDerivedPoses() {
    if (START_POSES.isEmpty()) {
      START_POSES.add(AB_START);
      START_POSES.add(CD_START);
      START_POSES.add(KL_START);
    }
    if (LEFT_POSES.isEmpty()) {
      LEFT_POSES.add(AB_LEFT);
      LEFT_POSES.add(CD_LEFT);
      LEFT_POSES.add(KL_LEFT);
    }
    if (RIGHT_POSES.isEmpty()) {
      RIGHT_POSES.add(AB_RIGHT);
      RIGHT_POSES.add(CD_RIGHT);
      RIGHT_POSES.add(KL_RIGHT);
    }
  }
}
