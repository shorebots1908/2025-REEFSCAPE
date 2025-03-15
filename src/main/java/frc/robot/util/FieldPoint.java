package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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

  public static void initScorePoses() {
    scorePoses.add(REEF_AB);
    scorePoses.add(REEF_CD);
    scorePoses.add(REEF_EF);
    scorePoses.add(REEF_GH);
    scorePoses.add(REEF_IJ);
    scorePoses.add(REEF_KL);
    scorePoses.add(CORAL_PROCESSOR1);
    scorePoses.add(CORAL_PROCESSOR2);

  }

}
