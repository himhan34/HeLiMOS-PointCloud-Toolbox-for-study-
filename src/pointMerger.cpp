#include "PointCloudProcessor.h"

// 프로그램의 시작점인 main 함수 정의
int main(int argc, char **argv)
{
    // YAML 설정 파일을 로드합니다.
    YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");

    // Ouster의 PointCloudProcessor 객체를 생성하고 초기화합니다.
    PointCloudProcessor OusterProcessor(config, 0);
    OusterProcessor.loadTrajectoryAndPoses(); // 경로와 자세를 로드합니다.

    // Velodyne의 PointCloudProcessor 객체를 생성하고 초기화합니다.
    PointCloudProcessor VelodyneProcessor(config, 1);
    VelodyneProcessor.loadTrajectoryAndPoses(); // 경로와 자세를 로드합니다.

    // Avia (Livox)의 PointCloudProcessor 객체를 생성하고 초기화합니다.
    PointCloudProcessor AviaProcessor(config, 2);
    AviaProcessor.loadTrajectoryAndPoses(); // 경로와 자세를 로드합니다.

    // Aeva의 PointCloudProcessor 객체를 생성하고 초기화합니다.
    PointCloudProcessor AevaProcessor(config, 3);
    AevaProcessor.loadTrajectoryAndPoses(); // 경로와 자세를 로드합니다.

    // 타임스탬프 리스트를 불러옵니다.
    const auto &ts_o = OusterProcessor.timestamp_lists_;
    const auto &ts_v = VelodyneProcessor.timestamp_lists_;
    const auto &ts_a  = AviaProcessor.timestamp_lists_;
    const auto &ts_e = AevaProcessor.timestamp_lists_;

    // 각 센서의 첫 번째 타임스탬프를 출력합니다.
    std::cout << std::fixed << OusterProcessor.timestamp_lists_[0] << std::endl;
    std::cout << VelodyneProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AviaProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AevaProcessor.timestamp_lists_[0] << std::endl;

    // 타임스탬프 차이가 일정 범위 내에 있는지 확인하는 람다 함수 정의
    auto isDifferenceWithin = [](long long a, long long b, double diff_in_sec) {
      auto diff = static_cast<double>(std::abs(a - b)) / 1e9; // 나노초를 초 단위로 변환
      return std::abs(diff) <= diff_in_sec; // 주어진 차이 내에 있는지 확인
    };

    // 가장 작은 크기와 가장 큰 크기를 찾는 람다 함수 정의
    auto findMinMaxSize = [](const std::vector<long long> &ts_o,
                           const std::vector<long long> &ts_v,
                           const std::vector<long long> &ts_a,
                           const std::vector<long long> &ts_e) {
      size_t minSize = std::min({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()}); // 최소 크기 찾기
      size_t maxSize = std::max({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()}); // 최대 크기 찾기

      return std::make_tuple(minSize, maxSize); // 결과를 튜플로 반환
  };

  // Ouster가 가장 먼저 시작하는지 확인합니다.
  if (ts_o[0] < ts_v[0] && ts_o[0] < ts_a[0] && ts_o[0] < ts_e[0]) {
    std::cout << std::fixed << "Check Ouster comes first!" << std::endl;
  } else {
    // Ouster가 먼저 오지 않으면 예외를 발생시킵니다.
    std::runtime_error("Currently, only KAIST05 is supported while strongly assuming that the Ouster comes first");
  }

  // 시간 차이 허용 범위를 설정합니다.
  double diff_in_sec = 0.08;

  // 최소 크기와 최대 크기를 가져옵니다.
  const auto &[minSize, maxSize] = findMinMaxSize(ts_o, ts_v, ts_a, ts_e);

  // 모든 타임스탬프가 주어진 범위 내에 있는지 확인합니다.
  for (int i = 0; i < minSize; ++i) {
    if (isDifferenceWithin(ts_o[i], ts_v[i], diff_in_sec) && isDifferenceWithin(ts_o[i], ts_a[i], diff_in_sec)
      && isDifferenceWithin(ts_o[i], ts_e[i], diff_in_sec)) {
      continue; // 범위 내에 있으면 계속 진행
    } else {
      // 범위를 벗어나면 예외를 발생시킵니다.
      std::runtime_error("Something's wrong. Timestamp is not matched :(");
    }
  }

  // 설정 파일에서 저장 형식을 불러옵니다.
  std::string saveAs = config["Save"]["saveAs"].as<std::string>();
  // 병합된 데이터를 저장할 경로를 설정합니다.
  std::string mergedSavePath = config["Path"]["savePath"].as<std::string>() + "Merged/";
  std::string mergedSavePathLiDAR = mergedSavePath + "velodyne/";
  std::string mergedSavePathPose = mergedSavePath + "poses.txt";
  std::string mergedSavePathCalib = mergedSavePath + "calib.txt";
  std::ofstream foutMergedPose;
  std::ofstream foutMergedCalib;
  pcl::PCDWriter pcdWriter;

  // 병합된 데이터를 저장할 디렉토리가 없으면 생성합니다.
  if (!std::filesystem::exists(mergedSavePath)) {
    std::cout << "Creating a directory: " << mergedSavePath << std::endl;
    std::filesystem::create_directory(mergedSavePath);
  }
  if (!std::filesystem::exists(mergedSavePathLiDAR)) {
    std::cout << "Creating a directory: " << mergedSavePathLiDAR << std::endl;
    std::filesystem::create_directory(mergedSavePathLiDAR);
  }

  // 1. 보정 파일을 작성합니다.
  writeCalibrationFile(mergedSavePathCalib);

  // 2. 포즈 파일을 작성합니다.
  foutMergedPose.open(mergedSavePathPose);

  // 최대 크기만큼 반복합니다.
  for (int i = 0; i < maxSize; ++i) {
    // 포인트 클라우드 객체를 생성합니다.
    pcl::PointCloud<pcl::PointXYZI>::Ptr ousterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulatedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. 각 포인트 클라우드를 Ouster 프레임으로 변환합니다.
    const Eigen::Matrix4f T_o = OusterProcessor.gt_poses_.at(i);
    foutMergedPose << T_o(0, 0) << " " << T_o(0, 1) << " " << T_o(0, 2) << " " << T_o(0, 3) << " "
                   << T_o(1, 0) << " " << T_o(1, 1) << " " << T_o(1, 2) << " " << T_o(1, 3) << " "
                   << T_o(2, 0) << " " << T_o(2, 1) << " " << T_o(2, 2) << " " << T_o(2, 3) << std::endl;

    pcl::PointXYZI minPt, maxPt;

    // 3-1. Ouster 포인트 클라우드를 병합합니다.
    loadCloud(i, OusterProcessor.savePathLiDAR, "bin", *ousterCloud);
    *accumulatedCloud = *ousterCloud;

    // 3-2. Velodyne 포인트 클라우드를 병합합니다.
    VelodyneProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    // 3-3. Avia (Livox) 포인트 클라우드를 병합합니다.
    AviaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    // 3-4. Aeva 포인트 클라우드를 병합합니다.
    AevaProcessor.getTransformedCloud(i, T_o, *transformedCloud);
    *accumulatedCloud += *transformedCloud;

    // 포인트 클라우드를 다운샘플링합니다.
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);

    sor.setInputCloud(accumulatedCloud);
    sor.setLeafSize(0.1, 0.1, 0.1); // 리프 크기를 설정합니다.
    sor.filter(*sampledCloud); // 필터를 적용합니다.

    std::cout << accumulatedCloud->points.size() << " -----> " << sampledCloud->size() << std::endl;
    
    // 지정된 형식에 따라 포인트 클라우드를 저장합니다.
    if (saveAs == "pcd") {
      pcdWriter.writeBinary(mergedSavePathLiDAR + padZeros(i, 6) + ".pcd", *accumulatedCloud);
    } else if (saveAs == "bin") {
      saveToBinFile(mergedSavePathLiDAR + padZeros(i, 6) + ".bin", *accumulatedCloud);
    }
    std::cout << "Saved " << i << "th synced frame" << std::endl;
  }

  return 0; // 프로그램 종료
}

