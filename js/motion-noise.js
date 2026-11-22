// Noise the filter adds in the MOVE step: how much it distrusts its own odometry.
// [a1, a2, a3, a4] of the odometry motion model (Thrun et al., Probabilistic Robotics, ch. 5):
// a1 rotation from rotation, a2 rotation from distance, a3 distance from distance, a4 distance from rotation.
// Tuned so the laser keeps the robot within a few centimetres from tile up to a thick rug.
export function motionAlphas(gyro, distrust) {
  const d = distrust;
  return gyro
    ? [0.02 * d, 0.004 * d, 0.2 * d, 0.01 * d]
    : [0.08 * d, 0.2 * d, 0.2 * d, 0.03 * d];
}
