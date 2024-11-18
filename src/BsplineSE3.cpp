#include "BsplineSE3.h"
#include <vector>
#include <Eigen/Dense>
#include <dirent.h>

using namespace ov_core;

// BsplineSE3 클래스의 feed_trajectory 함수입니다. 경로 데이터를 입력받아 B-스플라인 보간을 수행합니다.
void BsplineSE3::feed_trajectory(std::vector<Eigen::VectorXd> traj_points)
{
  time_feed = omp_get_wtime();  // 현재 시간을 저장합니다.

  // 평균 주파수를 계산하여 균일한 시간 간격을 설정합니다.
  double sumdt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++)
  {
    sumdt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);  // 인접한 점들 간의 시간 차이를 누적합니다.
  }
  dt = sumdt / (traj_points.size() - 1);  // 평균 시간 간격을 계산합니다.
  dt = (dt < 0.01) ? 0.01 : 0.01;  // 최소 시간 간격을 0.01로 설정합니다.

  // 모든 경로 점들을 SE(3) 행렬로 변환합니다.
  // traj_points는 [타임스탬프, p_IinG, q_GtoI] 형식을 가집니다.
  AlignedEigenMat4d trajectory_points;
  for (size_t i = 0; i < traj_points.size() - 1; i++)
  {
    Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();  // SE(3) 행렬을 단위 행렬로 초기화합니다.
    T_IinG.block(0, 0, 3, 3) = quat_2_Rot(traj_points.at(i).block(4, 0, 4, 1)).transpose();  // 쿼터니언을 회전 행렬로 변환 후 전치합니다.
    T_IinG.block(0, 3, 3, 1) = traj_points.at(i).block(1, 0, 3, 1);  // 위치 벡터를 설정합니다.
    trajectory_points.insert({traj_points.at(i)(0), T_IinG});  // 타임스탬프와 SE(3) 행렬을 삽입합니다.
  }

  // 가장 오래된 타임스탬프와 가장 최신 타임스탬프를 구합니다.
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto &pose : trajectory_points)
  {
    if (pose.first <= timestamp_min)
    {
      timestamp_min = pose.first;  // 가장 작은 타임스탬프를 설정합니다.
    }
    if (pose.first >= timestamp_min)
    {
      timestamp_max = pose.first;  // 가장 큰 타임스탬프를 설정합니다.
    }
  }

  // 스플라인 제어점을 생성합니다.
  double timestamp_curr = timestamp_min;
  while (true)
  {
    // 현재 시간에 대한 경계 포즈를 찾습니다.
    double t0, t1;
    Eigen::Matrix4d pose0, pose1;
    bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0, pose0, t1, pose1);

    // 경계 포즈를 찾지 못하면 데이터셋 끝에 도달한 것이므로 루프를 종료합니다.
    if (!success)
      break;

    // 선형 보간을 수행하고 제어점에 추가합니다.
    double lambda = (timestamp_curr - t0) / (t1 - t0);  // 보간 비율을 계산합니다.
    Eigen::Matrix4d pose_interp = exp_se3(lambda * log_se3(pose1 * Inv_se3(pose0))) * pose0;  // 보간된 포즈를 계산합니다.
    control_points.insert({timestamp_curr, pose_interp});  // 보간된 포즈를 제어점에 추가합니다.
    timestamp_curr += dt;  // 현재 시간을 증가시킵니다.
  }

  // 시스템의 시작 시간은 최소 두 개의 제어점이 필요하므로 2 * dt를 더합니다.
  timestamp_start = timestamp_min + 2 * dt;
}

// 특정 시간의 포즈를 얻는 함수입니다.
bool BsplineSE3::get_pose(double timestamp, Eigen::Quaterniond &q_GtoI, Eigen::Vector3d &p_IinG)
{
  time_feed = omp_get_wtime();  // 현재 시간을 저장합니다.

  // 원하는 시간의 경계 제어점을 찾습니다.
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  Eigen::Matrix3d R_GtoI;
  bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // 경계 제어점을 찾지 못하면 기본값을 설정하고 실패를 반환합니다.
  if (!success)
  {
    R_GtoI.setIdentity();  // 회전 행렬을 단위 행렬로 설정합니다.
    p_IinG.setZero();  // 위치 벡터를 0으로 설정합니다.
    return false;
  }

  // De Boor-Cox 보간 스칼라를 계산합니다.
  double DT = (t2 - t1);  // 제어점 간의 시간 간격입니다.
  double u = (timestamp - t1) / DT;  // 보간 변수 u를 계산합니다.
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);  // b0 스칼라를 계산합니다.
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);  // b1 스칼라를 계산합니다.
  double b2 = 1.0 / 6.0 * (u * u * u);  // b2 스칼라를 계산합니다.

  // 보간된 포즈를 계산합니다.
  Eigen::Matrix4d A0 = exp_se3(b0 * log_se3(Inv_se3(pose0) * pose1));  // 첫 번째 보간 행렬을 계산합니다.
  Eigen::Matrix4d A1 = exp_se3(b1 * log_se3(Inv_se3(pose1) * pose2));  // 두 번째 보간 행렬을 계산합니다.
  Eigen::Matrix4d A2 = exp_se3(b2 * log_se3(Inv_se3(pose2) * pose3));  // 세 번째 보간 행렬을 계산합니다.

  // 최종 보간된 포즈를 얻습니다.
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3);  // 회전 부분을 추출합니다.
  q_GtoI = R_GtoI;  // 회전 행렬을 쿼터니언으로 변환합니다.
  p_IinG = pose_interp.block(0, 3, 3, 1);  // 위치 벡터를 추출합니다.
  total_time += (omp_get_wtime() - time_feed);  // 총 시간을 업데이트합니다.

  return true;  // 성공을 반환합니다.
}

// 특정 시간의 경계 포즈를 찾는 함수입니다.
bool BsplineSE3::find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                     Eigen::Matrix4d &pose1)
{
  // 기본값을 설정합니다.
  t0 = -1;
  t1 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();

  // 경계 포즈를 찾기 위한 플래그입니다.
  bool found_older = false;
  bool found_newer = false;

  // 주어진 시간의 경계 포즈를 찾습니다.
  auto lower_bound = poses.lower_bound(timestamp);  // timestamp 또는 그 다음 timestamp를 찾습니다.
  auto upper_bound = poses.upper_bound(timestamp);  // 다음 timestamp를 찾습니다.

  if (lower_bound != poses.end())
  {
    // lower_bound가 주어진 timestamp인지 확인합니다.
    // 아니라면 이전 timestamp로 이동하여 경계가 되도록 설정합니다.
    if (lower_bound->first == timestamp)
    {
      found_older = true;
    }
    else if (lower_bound != poses.begin())
    {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end())
  {
    found_newer = true;  // upper_bound가 유효하다면 설정합니다.
  }

  // 이전 경계 포즈를 설정합니다.
  if (found_older)
  {
    t0 = lower_bound->first;
    pose0 = lower_bound->second;
  }

  // 이후 경계 포즈를 설정합니다.
  if (found_newer)
  {
    t1 = upper_bound->first

;
    pose1 = upper_bound->second;
  }

  // 타임스탬프를 확인합니다.
  if (found_older && found_newer)
    assert(t0 < t1);

  // 경계 포즈를 모두 찾았으면 true를 반환합니다.
  return (found_older && found_newer);
}

// 특정 시간의 경계 제어점을 찾는 함수입니다.
bool BsplineSE3::find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                              double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                              Eigen::Matrix4d &pose3)
{
  // 기본값을 설정합니다.
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();
  pose2 = Eigen::Matrix4d::Identity();
  pose3 = Eigen::Matrix4d::Identity();

  // 두 개의 경계 포즈를 가져옵니다.
  bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

  // 경계 포즈를 찾지 못하면 false를 반환합니다.
  if (!success)
    return false;

  // 이전과 이후의 포즈를 찾습니다.
  auto iter_t1 = poses.find(t1);
  auto iter_t2 = poses.find(t2);

  // t1이 첫 번째 타임스탬프인지 확인합니다.
  if (iter_t1 == poses.begin())
  {
    return false;
  }

  // 이전 포즈를 되돌리고, 이후 포즈를 앞으로 이동합니다.
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // 이후 포즈가 유효한지 확인합니다.
  if (iter_t3 == poses.end())
  {
    return false;
  }

  // 가장 오래된 포즈를 설정합니다.
  t0 = iter_t0->first;
  pose0 = iter_t0->second;

  // 가장 새로운 포즈를 설정합니다.
  t3 = iter_t3->first;
  pose3 = iter_t3->second;

  // 타임스탬프를 확인합니다.
  if (success)
  {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // 모든 경계 포즈를 찾았으면 true를 반환합니다.
  return success;
}
