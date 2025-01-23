// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOADIS16470 implements GyroIO {
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  // private final double yaw = m_gyro.getAngle();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  // private final double yawVelocity = m_gyro.getAccelZ();

  public GyroIOADIS16470() {
    // TODO
    m_gyro.calibrate();

    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(m_gyro.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_gyro.getAccelZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    // TODO
  }
}
