#include "PointCloudProcessor.h"

// 생성자: PointCloudProcessor 클래스의 객체를 초기화합니다.
PointCloudProcessor::PointCloudProcessor(YAML::Node &config, int sensorType)
{
  // 배너를 표시합니다.
  displayBanner();
  // 입력 데이터를 모읍니다.
  gatherInput(config, sensorType);
  // 입력 데이터를 표시합니다.
  displayInput();
  // 진행 바를 표시합니다.
  visualizer.progressBar(0, 1, "", false);
  // "loadTrajectoryAndPoses()" 함수를 불러오는 코드 (현재 주석 처리됨)
}

// 소멸자: PointCloudProcessor 클래스의 객체가 소멸될 때 호출됩니다.
PointCloudProcessor::~PointCloudProcessor() {}

// 배너를 표시하는 함수입니다.
void PointCloudProcessor::displayBanner()
{
  // 화면을 지웁니다.
#ifdef _WIN32
  system("cls"); // Windows에서는 "cls" 명령어를 사용하여 화면을 지웁니다.
#else
  system("clear"); // 다른 운영체제에서는 "clear" 명령어를 사용하여 화면을 지웁니다.
#endif

  // 배너의 너비를 고정합니다.
  const int bannerWidth = 80; // 필요한 경우 이 너비를 조정합니다.

  // 배너의 제목을 정의합니다.
  const std::string title = "HeLiMOS Point Cloud Undistortion and Merging Tool";
  
  // 배너의 설명을 정의합니다.
  const std::string description = "This file is a utility designed to undistort the point clouds, "
                                  "from the HeLiPR dataset in order to create the HeLiMOS dataset. "
                                  "By running this file, you will generate point cloud files "
                                  "that conform to the SemanticKITTI format, as well as the poses.txt and calib.txt files.";

  // 입력 및 출력 정보에 대한 설명을 정의합니다.
  const std::string inputInfo = "Input: {timestamp}.bin files, TUM format gt poses file.\n"
                                "Output: {index}.bin files, SemanticKITTI format gt poses file, calib.txt file.\n";

  // 유지 관리자에 대한 정보를 정의합니다.
  const std::string maintainerInfo = "Maintainer: Seoyeon Jang (9uantum01@kaist.ac.kr, Urban Robotics Lab), Revised: 2024/08/01";

  // 제목의 여백을 계산합니다.
  int titlePadding = (bannerWidth - title.length()) / 2;
  titlePadding = titlePadding < 0 ? 0 : titlePadding; // 여백이 음수가 되지 않도록 설정합니다.

  // 고정된 너비로 배너를 표시합니다.
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n"; // 배너의 상단 경계선을 출력합니다.
  std::cout << std::string(titlePadding, ' ') << yellow << title << reset << "\n"; // 제목을 가운데 정렬하여 출력합니다.
  std::cout << cyan << std::string(bannerWidth, '*') << reset << "\n"; // 제목 아래에 경계선을 출력합니다.

  // 설명과 입력 정보를 래핑하여 출력합니다.
  std::string combinedInfo = description;
  std::istringstream words(combinedInfo); // 설명을 단어로 나눕니다.
  std::string word;
  std::string line;
  while (words >> word) // 각 단어를 순차적으로 처리합니다.
  {
    // 현재 줄에 단어를 추가했을 때 배너 너비를 초과하는지 확인합니다.
    if (line.length() + word.length() + 1 > bannerWidth)
    {
      std::cout << line << std::endl; // 현재 줄을 출력합니다.
      line = word; // 새 줄에 단어를 추가합니다.
    }
    else
    {
      line += (line.empty() ? "" : " ") + word; // 단어를 현재 줄에 추가합니다.
    }
  }
  if (!line.empty())
    std::cout << line << std::endl; // 마지막 줄을 출력합니다.

  // 정보 구분선을 출력합니다.
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << inputInfo << "\n"; // 입력 및 출력 정보를 출력합니다.
  std::cout << cyan << std::string(bannerWidth, '-') << reset << "\n";
  std::cout << maintainerInfo << "\n"; // 유지 관리자 정보를 출력합니다.
  std::cout << cyan << std::string(bannerWidth, '=') << reset << "\n\n"; // 배너의 하단 경계선을 출력합니다.
}


void PointCloudProcessor::gatherInput(YAML::Node &config, int sensorType)
{
  // yaml 파일에서 config를 불러옵니다.
  std::string sensorString;
  int LiDARTypeInt;

  // sensorType에 따라 LiDAR 정보를 설정합니다.
  switch (sensorType)
  {
  case 0:
    sensorString = "Ouster"; // 센서가 Ouster일 경우
    LiDARTypeInt = 0;
    break;
  case 1:
    sensorString = "Velodyne"; // 센서가 Velodyne일 경우
    LiDARTypeInt = 1;
    break;
  case 2:
    sensorString = "Avia"; // 센서가 Avia일 경우
    LiDARTypeInt = 2;
    break;
  case 3:
    sensorString = "Aeva"; // 센서가 Aeva일 경우
    LiDARTypeInt = 3;
    break;
  }
  // * 경로 설정 및 로드
  // YAML 파일에서 경로를 불러옵니다.
  binPath = config["Path"]["binPath"].as<std::string>() + sensorString + "/";
  trajPath = config["Path"]["trajPath"].as<std::string>() + sensorString + "_gt.txt";
  savePath = config["Path"]["savePath"].as<std::string>() + sensorString + "/";
  savePathLiDAR = savePath + "velodyne/";
  savePathPose = savePath + "poses.txt";
  savePathCalib = savePath + "calib.txt";
  savePathLabel = savePath + "labels/";

  // * 저장을 위한 파라미터 설정
  undistortFlag = config["Undistort"]["undistortFlag"].as<bool>();
  numIntervals = config["Undistort"]["numIntervals"].as<int>();
  downSampleFlag = config["Save"]["downSampleFlag"].as<bool>();
  downSampleVoxelSize = config["Save"]["downSampleVoxelSize"].as<float>();
  downsamplePointSize = config["Save"]["downsamplePointSize"].as<int>();
  normalizeFlag = config["Save"]["normalizeFlag"].as<bool>();
  saveName = config["Save"]["saveName"].as<std::string>();
  saveAs = config["Save"]["saveAs"].as<std::string>();
  cropFlag = config["Save"]["cropFlag"].as<bool>();
  cropSize = config["Save"]["cropSize"].as<float>();

  distanceThreshold = config["Save"]["distanceThreshold"].as<int>();
  accumulatedSize = config["Save"]["accumulatedSize"].as<int>();
  accumulatedStep = config["Save"]["accumulatedStep"].as<int>();

  // 입력 경로를 확인합니다.
  if (!std::filesystem::exists(binPath) || binPath.substr(binPath.size() - 1) != "/")
  {
    std::cout << red << "The path does not exist or the path is not end with folder/. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // 경로가 존재하는지 및 파일 형식이 올바른지 확인합니다.
  if (!std::filesystem::exists(trajPath) || trajPath.substr(trajPath.size() - 4) != ".txt")
  {
    std::cout << red << "The path does not exist or the file is not txt. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // 저장 경로가 존재하는지 확인하고 없을 경우 디렉터리를 생성합니다.
  if (!std::filesystem::exists(savePath))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    if (!std::filesystem::create_directory(savePath))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  // LiDAR 저장 경로가 존재하지 않을 경우 디렉터리를 생성합니다.
  if (!std::filesystem::exists(savePathLiDAR))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    if (!std::filesystem::create_directory(savePathLiDAR))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  // 레이블 경로가 존재하지 않을 경우 디렉터리를 생성합니다.
  if (!std::filesystem::exists(savePathLabel))
  {
    std::cout << yellow << "The path does not exist. Creating a new directory." << reset << std::endl;
    if (!std::filesystem::create_directory(savePathLabel))
    {
      std::cout << red << "Failed to create the directory. Please check config.yaml." << reset << std::endl;
      exit(0);
    }
  }

  // numIntervals가 1 이하일 경우 오류를 출력합니다.
  if (numIntervals <= 1)
  {
    std::cout << red << "numIntervals should be greater than 1. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // accumulatedSize가 0 이하일 경우 오류를 출력합니다.
  if (accumulatedSize <= 0)
  {
    std::cout << red << "accumulatedSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // accumulatedStep이 1 미만일 경우 오류를 출력합니다.
  if (accumulatedStep < 1)
  {
    std::cout << red << "accumulatedStep should be greater than or equal to 1. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // 다운샘플링 파라미터가 잘못된 경우 오류를 출력합니다.
  if (downSampleFlag && downSampleVoxelSize <= 0 && downsamplePointSize <= 0)
  {
    std::cout << red << "downSampleVoxelSize and downsamplePointSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // saveName이 "Index"나 "Timestamp"가 아닌 경우 오류를 출력합니다.
  if (saveName != "Index" && saveName != "Timestamp")
  {
    std::cout << red << "saveName should be Index or Timestamp. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // saveAs가 "bin"이나 "pcd"가 아닌 경우 오류를 출력합니다.
  if (saveAs != "bin" && saveAs != "pcd")
  {
    std::cout << red << "saveAs should be bin or pcd. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // cropFlag가 활성화되어 있으면서 cropSize가 0 이하일 경우 오류를 출력합니다.
  if (cropFlag && cropSize <= 0)
  {
    std::cout << red << "cropSize should be greater than 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // distanceThreshold가 0 미만일 경우 오류를 출력합니다.
  if (distanceThreshold < 0)
  {
    std::cout << red << "distanceThreshold should be greater than or equal to 0. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // LiDARTypeInt가 범위를 벗어난 경우 오류를 출력합니다.
  if (LiDARTypeInt < 0 || LiDARTypeInt > 3)
  {
    std::cout << red << "LiDAR should be 0, 1, 2, or 3. Please check config.yaml." << reset << std::endl;
    exit(0);
  }

  // LiDAR 타입을 설정합니다.
  LiDAR = static_cast<LiDARType>(LiDARTypeInt);
}

void PointCloudProcessor::displayInput()
{
  const int width = 20; // 첫 번째 열의 너비를 설정합니다.

  // 구분선을 출력합니다.
  std::cout << green << std::string(40, '-') << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Parameter"
            << "Value" << reset << std::endl; // 열 제목을 출력합니다.
  std::cout << green << std::string(40, '-') << reset << std::endl; // 구분선을 출력합니다.
  
  // 각 매개변수를 출력합니다.
  std::cout << green << std::left << std::setw(width) << "binPath:" << binPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "trajPath:" << trajPath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "savePath:" << savePath << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "savePathLiDAR:" << savePathLiDAR << reset << std::endl;
  
  // LiDAR 타입 문자열을 변환하여 출력합니다.
  std::string lidarTypeStr = visualizer.lidarTypeToString(LiDAR);
  std::cout << green << std::left << std::setw(width) << "LiDAR:" << lidarTypeStr << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Distance Threshold:" << distanceThreshold << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Num Intervals:" << numIntervals << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Accumulated Size:" << accumulatedSize << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Accumulated Step:" << accumulatedStep << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Downsample Flag:" << (downSampleFlag ? "True" : "False") << reset << std::endl;
  
  // 다운샘플링이 활성화된 경우 추가 정보를 출력합니다.
  if (downSampleFlag)
  {
    std::cout << green << std::left << std::setw(width) << "Downsample Voxel Size:" << downSampleVoxelSize << reset << std::endl;
    std::cout << green << std::left << std::setw(width) << "Downsample Point Size:" << downsamplePointSize << reset << std::endl;
  }
  
  std::cout << green << std::left << std::setw(width) << "Normalize Flag:" << (normalizeFlag ? "True" : "False") << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Save Name:" << saveName << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Save As:" << saveAs << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Undistort Flag:" << (undistortFlag ? "True" : "False") << reset << std::endl;
  std::cout << green << std::left << std::setw(width) << "Crop Flag:" << (cropFlag ? "True" : "False") << reset << std::endl;
  
  // 크롭이 활성화된 경우 크기 정보를 출력합니다.
  if (cropFlag)
    std::cout << green << std::left << std::setw(width) << "Crop Size:" << cropSize << reset << std::endl;
  std::cout << std::endl;
}

void PointCloudProcessor::interpolate(double timestamp, double timeStart, double dt,
                                      const std::vector<Eigen::Quaterniond> &quaternions,
                                      const std::vector<Eigen::Vector3d> &positions,
                                      int numIntervals, Eigen::Quaterniond &qOut,
                                      Eigen::Vector3d &pOut)
{
  // 쿼터니언 및 위치 벡터에서 인덱스를 계산합니다.
  int idx = (timestamp - timeStart) / dt;
  if (idx < 0)
    idx = 0; // 인덱스가 0보다 작을 경우 0으로 설정합니다.
  if (idx >= numIntervals - 1)
    idx = numIntervals - 2; // 인덱스가 범위를 초과할 경우 최대값으로 설정합니다.

  // 보간 계수를 계산합니다.
  double alpha = (timestamp - (timeStart + idx * dt)) / dt;

  // 쿼터니언을 SLERP(Spherical Linear Interpolation) 방식으로 보간합니다.
  qOut = quaternions[idx].slerp(alpha, quaternions[idx + 1]);

  // 위치를 선형 보간합니다.
  pOut = (1 - alpha) * positions[idx] + alpha * positions[idx + 1];
}

template <class T>
void PointCloudProcessor::processPoint(T &point, double timestamp, double timeStart, double dt, Eigen::Quaterniond &qStart, Eigen::Vector3d &pStart,
                                       const std::vector<Eigen::Quaterniond> &quaternions,
                                       const std::vector<Eigen::Vector3d> &positions,
                                       int numIntervals, Eigen::Quaterniond &qOut,
                                       Eigen::Vector3d &pOut)
{
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pScan;
  
  // 보간을 수행하여 스캔 시점의 쿼터니언과 위치를 얻습니다.
  interpolate(timestamp, timeStart, dt, quaternions, positions, numIntervals, qScan, pScan);
  
  // 포인트를 변환합니다.
  Eigen::Vector3d transformedPoint(point.x, point.y, point.z);
  transformedPoint = qStart * ((qScan * transformedPoint + pScan) - pStart);
  
  // 변환된 좌표를 포인트에 반영합니다.
  point.x = transformedPoint(0);
  point.y = transformedPoint(1);
  point.z = transformedPoint(2);
}


void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<OusterPointXYZIRT> &cloud)
{
  // 파일 스트림을 선언합니다.
  std::ifstream file;
  // 이진 형식으로 파일을 엽니다.
  file.open(filename, std::ios::in | std::ios::binary);

  // 파일의 끝까지 반복합니다.
  while (!file.eof())
  {
    // OusterPointXYZIRT 포인트 객체를 선언합니다.
    OusterPointXYZIRT point;
    // 각 데이터 필드를 이진 형식으로 읽어옵니다.
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
    // 포인트를 포인트 클라우드에 추가합니다.
    cloud.push_back(point);
  }
  // 파일을 닫습니다.
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<PointXYZIRT> &cloud)
{
  // 파일 스트림을 선언합니다.
  std::ifstream file;
  // 이진 형식으로 파일을 엽니다.
  file.open(filename, std::ios::in | std::ios::binary);

  // 파일의 끝까지 반복합니다.
  while (!file.eof())
  {
    // PointXYZIRT 포인트 객체를 선언합니다.
    PointXYZIRT point;
    // 각 데이터 필드를 이진 형식으로 읽어옵니다.
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
    // 포인트를 포인트 클라우드에 추가합니다.
    cloud.push_back(point);
  }
  // 파일을 닫습니다.
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<LivoxPointXYZI> &cloud)
{
  // 파일 스트림을 선언합니다.
  std::ifstream file;
  // 이진 형식으로 파일을 엽니다.
  file.open(filename, std::ios::in | std::ios::binary);

  // 파일의 끝까지 반복합니다.
  while (!file.eof())
  {
    // LivoxPointXYZI 포인트 객체를 선언합니다.
    LivoxPointXYZI point;
    // 각 데이터 필드를 이진 형식으로 읽어옵니다.
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    uint8_t intensity;
    file.read(reinterpret_cast<char *>(&intensity), sizeof(uint8_t));
    // uint8_t 데이터를 float로 변환하여 설정합니다.
    point.intensity = intensity;

    // 태그 및 라인 필드를 읽어옵니다.
    file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
    file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));

    // 시야각 내에 있는 포인트만 추가합니다.
    if (isInFOV(point.x, point.y, point.z))
      cloud.push_back(point);
  }
  // 파일을 닫습니다.
  file.close();
}

void PointCloudProcessor::readBinFile(const std::string &filename, pcl::PointCloud<AevaPointXYZIRT> &cloud, double timeStart)
{
  // 파일 스트림을 선언합니다.
  std::ifstream file;
  // 이진 형식으로 파일을 엽니다.
  file.open(filename, std::ios::in | std::ios::binary);

  // 파일의 끝까지 반복합니다.
  while (!file.eof())
  {
    // AevaPointXYZIRT 포인트 객체를 선언합니다.
    AevaPointXYZIRT point;
    // 각 데이터 필드를 이진 형식으로 읽어옵니다.
    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
    file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
    file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
    // 특정 시간 조건에 따라 추가적인 읽기를 수행합니다.
    if (timeStart > 1691936557946849179 / 1e9)
      file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    // 포인트를 포인트 클라우드에 추가합니다.
    cloud.push_back(point);
  }
  // 파일을 닫습니다.
  file.close();
}

template <class T>
void PointCloudProcessor::accumulateScans(std::vector<pcl::PointCloud<T>> &vecCloud, Point3D &lastPoint)
{
  // PCD 작성자를 선언합니다.
  pcl::PCDWriter pcdWriter;
  // 누적 포인트 클라우드를 선언합니다.
  pcl::PointCloud<T> accumulatedCloud;

  // vecCloud가 누적 크기와 같을 때 실행합니다.
  if (vecCloud.size() == accumulatedSize)
  {
    // 마지막 포인트와 첫 번째 스캔 포인트 사이의 거리를 확인합니다.
    if (euclidean_distance(lastPoint, scanPoints[0]) > distanceThreshold)
    {
      // 마지막 포인트를 갱신합니다.
      lastPoint = scanPoints[0];
      // 모든 포인트 클라우드를 반복합니다.
      for (int i = 0; i < accumulatedSize; i++)
      {
        // 역순으로 포인트를 반복합니다.
        auto pclIter = vecCloud[i].points.end() - 1;
        for (; pclIter != vecCloud[i].points.begin(); pclIter--)
        {
          T point = *pclIter;
          // 포인트를 변환합니다.
          Eigen::Vector3d transformedPoint(pclIter->x, pclIter->y, pclIter->z);
          transformedPoint = scanQuat[0].conjugate() * ((scanQuat[i] * transformedPoint + scanTrans[i]) - scanTrans[0]);
          point.x = transformedPoint(0);
          point.y = transformedPoint(1);
          point.z = transformedPoint(2);

          // 크롭 플래그가 설정된 경우 크롭 영역 내의 포인트만 유지합니다.
          if (cropFlag && (point.x * point.x + point.y * point.y + point.z * point.z) > cropSize * cropSize)
            continue;

          // 포인트를 누적 포인트 클라우드에 추가합니다.
          accumulatedCloud.push_back(point);
        }
      }

      // 누적 포인트 클라우드를 샘플링합니다.
      pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
      copyPointCloud(accumulatedCloud, *sampledCloud);

      // 다운샘플링 플래그가 설정된 경우 다운샘플링을 수행합니다.
      if (downSampleFlag)
      {
        if (downsamplePointSize > 0)
          downsample_to_target_size(*sampledCloud, downsamplePointSize);
        else
          down_sampling_voxel(*sampledCloud, downSampleVoxelSize);
      }

      // 정규화 플래그가 설정된 경우 포인트 클라우드를 정규화합니다.
      if (normalizeFlag)
        normalizePointCloud(sampledCloud, cropSize);

      // 저장 파일 이름을 설정합니다.
      if (saveName == "Index")
        saveFile = savePathLiDAR + padZeros(keyIndex - accumulatedSize, 6);
      else if (saveName == "Timestamp")
        saveFile = savePathLiDAR + scanTimestamps[0];

      // 진행 상황을 표시합니다.
      visualizer.progressBar(keyIndex, numBins, saveFile + "." + saveAs, true);
      // 파일을 저장합니다.
      if (saveAs == "bin")
      {
        saveToBinFile(saveFile + ".bin", *sampledCloud);
      }
      else if (saveAs == "pcd")
        pcdWriter.writeBinary(saveFile + ".pcd", *sampledCloud);
    }

    // 누적된 스캔 데이터를 제거합니다.
    for (int i = 0; i < accumulatedStep; i++)
    {
      vecCloud.erase(vecCloud.begin());
      scanQuat.erase(scanQuat.begin());
      scanTrans.erase(scanTrans.begin());
      scanTimestamps.erase(scanTimestamps.begin());
      scanPoints.erase(scanPoints.begin());
    }
  }
}


void PointCloudProcessor::processFile(const std::string &filename)
{
  // OusterPointXYZIRT 타입의 포인트 클라우드 포인터 생성
  pcl::PointCloud<OusterPointXYZIRT>::Ptr scanOuster(new pcl::PointCloud<OusterPointXYZIRT>);
  // PointXYZIRT 타입의 Velodyne 포인트 클라우드 포인터 생성
  pcl::PointCloud<PointXYZIRT>::Ptr scanVelodyne(new pcl::PointCloud<PointXYZIRT>);
  // LivoxPointXYZI 타입의 Livox 포인트 클라우드 포인터 생성
  pcl::PointCloud<LivoxPointXYZI>::Ptr scanLivox(new pcl::PointCloud<LivoxPointXYZI>);
  // AevaPointXYZIRT 타입의 Aeva 포인트 클라우드 포인터 생성
  pcl::PointCloud<AevaPointXYZIRT>::Ptr scanAeva(new pcl::PointCloud<AevaPointXYZIRT>);

  // 시작 지점의 회전과 위치를 저장하는 변수 선언
  Eigen::Quaterniond qStart;
  Eigen::Quaterniond qScan;
  Eigen::Vector3d pStart;
  Eigen::Vector3d pScan;

  // 여러 구간의 회전과 위치를 저장하는 벡터 생성
  std::vector<Eigen::Quaterniond> quaternions(numIntervals);
  std::vector<Eigen::Vector3d> positions(numIntervals);

  // 파일명에서 타임스탬프를 추출
  std::size_t startPos = filename.find_last_of("/") + 1;
  std::size_t endPos = filename.find_last_of(".");
  std::string timestampStr = filename.substr(startPos, endPos - startPos);

  // 타임스탬프를 초 단위로 변환
  double timeStart = std::stod(timestampStr) / 1e9;

  // 타임스탬프가 궤적 범위를 벗어났는지 확인
  if (timeStart < trajPoints0 || timeStart > trajPoints  + 1 메시지 출력 후 종료
    std::cout << red << "The timestamp is out of range. Please check the trajectory file and bin files." << reset << std::endl;
    exit(0);
  }

  // B-spline SE3 궤적에서 포즈를 얻기
  bool success_field = bsplineSE3.get_pose(timeStart, qStart, pStart);

  // 초기 포인트에 대해서만 처리
  if (!success_field)
  {
    // 타임스탬프가 궤적 구간 내에 속하는 인덱스를 찾음
    int idx = 0;
    for (int i = 0; i < trajPoints.size() - 1; i++)
    {
      if (timeStart >= trajPoints  && timeStart <= trajPoints ) {
        idx = i;
           }
    }
    // 비율(alpha)에 따라 회전과 위치를 보간
    double alpha = (timeStart - trajPoints ) / (trajPoints  - trajPoints );
    Eigen:iond q0 = Eigend(trajPoints ints , trajPoints , trajPoints );
    Eigen::Quaterniond q1 :Quaterniondnts , trajPorajPoints , ts );
    qStart = q0.slerp(alpha, q1);
    pStart = (1 - aloints[idx].b, 1) + alphas[idx + 1].b, 1);
  }

  // LiDAR 타입에 따라 적절한 파일 읽기 함수 호출
  switch (LiDAR)
  {
  case OUSTER:
    readBinFile(filename, *scanOuster);
    break;
  case VELODYNE:
    readBinFile(filename, *scanVelodyne);
    break;
  case LIVOX:
    readBinFile(filename, *scanLivox);
    break;
  case AEVA:
    readBinFile(filename, *scanAeva, timeStart);
    break;
  }

  // 현재 포인트의 위치 설정
  Point3D currPoint;
  currPoint.x = pStart(0);
  currPoint.y = pStart(1);
  currPoint.z = pStart(2);

  // 성공한 경우 포인트와 관련 데이터를 저장
  scanPoints.push_back(currPoint);
  scanQuat.push_back(qStart);
  scanTrans.push_back(pStart);
  scanTimestamps.push_back(timestampStr);

  // 회전값을 켤레로 변경
  qStart = qStart.conjugate();
  double timeEnd = timeStart + 0.105; // 끝나는 시간 설정
  double dt = (timeEnd - timeStart) / numIntervals;
  bool exitFlag = false;

  // 각 구간에 대해 B-spline SE3 궤적에서 포즈를 얻기
  for (int i = 0; i < numIntervals; ++i)
  {
    double t = timeStart + i * dt;
    if (!bsplineSE3.get_pose(t, quaternions[i], positions[i]))
    {
      exitFlag = true;
    }
  }

  // LiDAR 타입에 따라 undistort 처리 및 포인트 저장
  switch (LiDAR)
  {
  case OUSTER:
    if (!exitFlag && undistortFlag)
    {
      // 병렬 처리를 통해 포인트의 타임스탬프와 보정 처리
      #pragma omp parallel for
      for (auto &point : scanOuster->points)
      {
        double timestamp = timeStart + point.t / float(1000000000);
        processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
      }
    }
    vecOuster.push_back(*scanOuster);
    break;
  case VELODYNE:
    if (!exitFlag && undistortFlag)
    {
      #pragma omp parallel for
      for (auto &point : scanVelodyne->points)
      {
        double timestamp = timeStart + point.time;
        processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
      }
    }
    vecVelodyne.push_back(*scanVelodyne);
    break;
  case LIVOX:
    if (!exitFlag && undistortFlag)
    {
      #pragma omp parallel for
      for (auto &point : scanLivox->points)
      {
        double timestamp = timeStart + point.offset_time / float(1000000000);
        processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
      }
    }
    vecLivox.push_back(*scanLivox);
    break;
  case AEVA:
    if (!exitFlag && undistortFlag)
    {
      #pragma omp parallel for
      for (auto &point : scanAeva->points)
      {
        double timestamp = timeStart + point.time_offset_ns / float(1000000000);
        processPoint(point, timestamp, timeStart, dt, qStart, pStart, quaternions, positions, numIntervals, qScan, pScan);
      }
    }
    vecAeva.push_back(*scanAeva);
    break;
  }

  // 키 인덱스 증가
  keyIndex++;
}
}


void PointCloudProcessor::loadAndProcessBinFiles()
{
  // 디렉토리 포인터와 디렉토리 항목 포인터 선언
  DIR *dir;
  struct dirent *ent;
  // .bin 파일 이름을 저장할 벡터 선언
  std::vector<std::string> binFiles;

  // 디렉토리를 열고 성공적으로 열렸는지 확인
  if ((dir = opendir(binPath.c_str())) != NULL)
  {
    // 디렉토리에서 파일들을 순차적으로 읽음
    while ((ent = readdir(dir)) != NULL)
    {
      std::string filename = ent->d_name;
      // 파일 이름이 ".bin"으로 끝나는 경우 벡터에 추가
      if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bin")
      {
        binFiles.push_back(filename);
      }
    }
    closedir(dir); // 디렉토리 닫기
  }
  
  // 캘리브레이션 파일 작성
  writeCalibrationFile(savePathCalib);
  foutPose.open(savePathPose); // 포즈 파일 열기

  // .bin 파일 이름을 정렬
  std::sort(binFiles.begin(), binFiles.end());
  // 마지막 포인트를 초기화
  Point3D lastPoint;
  lastPoint.x = -999;
  lastPoint.y = -999;
  lastPoint.z = -999;
  numBins = binFiles.size(); // .bin 파일 개수를 저장

  // 각 파일에 대해 처리
  for (const std::string &filename : binFiles)
  {
    // 진행 상황을 시각적으로 표시
    visualizer.progressBar(keyIndex, binFiles.size(), filename, false);
    processFile(binPath + filename); // 파일 처리

    // LiDAR 타입에 따라 포인트 클라우드를 누적
    switch (LiDAR)
    {
    case OUSTER:
      accumulateScans(vecOuster, lastPoint);
      break;
    case VELODYNE:
      accumulateScans(vecVelodyne, lastPoint);
      break;
    case LIVOX:
      accumulateScans(vecLivox, lastPoint);
      break;
    case AEVA:
      accumulateScans(vecAeva, lastPoint);
      break;
    }
    
    // 키 인덱스가 누적 크기보다 작은 경우 다음으로 넘어감
    if(keyIndex - accumulatedSize < 0)
      continue; // * 리팩토링이 필요함

    // 현재 키 인덱스에 대한 변환 행렬을 가져옴
    Eigen::Matrix4f tf4x4 = gt_poses_[keyIndex - accumulatedSize];
    // 변환 행렬을 파일에 저장
    foutPose << tf4x4(0, 0) << " " << tf4x4(0, 1) << " " << tf4x4(0, 2) << " " << tf4x4(0, 3) << " "
              << tf4x4(1, 0) << " " << tf4x4(1, 1) << " " << tf4x4(1, 2) << " " << tf4x4(1, 3) << " "
              << tf4x4(2, 0) << " " << tf4x4(2, 1) << " " << tf4x4(2, 2) << " " << tf4x4(2, 3) << std::endl; 
  }

  foutPose.close(); // 포즈 파일 닫기
}

void PointCloudProcessor::loadTrajectory()
{
  // 궤적 파일을 읽기 위해 줄 단위로 읽기
  std::string line;
  std::ifstream file(trajPath);

  // 파일에서 줄을 하나씩 읽어 처리
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    Eigen::VectorXd point(8);
    // 각 줄에서 포인트 데이터를 읽어들임
    if (!(iss >> point(0) >> point(1) >> point(2) >> point(3) >> point(4) >> point(5) >> point(6) >> point(7)))
    {
      break; // 읽기 실패 시 루프 종료
    }
    point(0) *= 1e-9; // 첫 번째 값을 초 단위로 변환
    trajPoints.push_back(point); // 궤적 포인트를 벡터에 추가
  }
  file.close(); // 파일 닫기
  bsplineSE3.feed_trajectory(trajPoints); // B-spline 궤적에 데이터 공급
}

void PointCloudProcessor::loadTrajectoryAndPoses()
{
  // 궤적과 포즈 데이터를 함께 로드
  std::string line;
  std::ifstream file(trajPath);

  gt_poses_.clear(); // 기존 포즈 데이터 초기화
  gt_poses_.reserve(100000); // 포즈 벡터의 메모리 예약

  // 파일에서 줄을 하나씩 읽어 처리
  while (std::getline(file, line))
  {
    // * 1. 포즈 로드
    const auto & [timestamp, pose] = splitLine(line, ' ');
    Eigen::Matrix4f tf4x4_sensor = Eigen::Matrix4f::Identity();
    vec2tf4x4(pose, tf4x4_sensor); // 벡터를 변환 행렬로 변환
    gt_poses_.push_back(tf4x4_sensor); // 포즈 추가
    timestamp_lists_.push_back(timestamp); // 타임스탬프 추가

    // * 2. 궤적 로드
    std::istringstream iss(line);
    Eigen::VectorXd point(8);
    if (!(iss >> point(0) >> point(1) >> point(2) >> point(3) >> point(4) >> point(5) >> point(6) >> point(7)))
    {
      break; // 읽기 실패 시 루프 종료
    }
    point(0) *= 1e-9; // 첫 번째 값을 초 단위로 변환
    trajPoints.push_back(point); // 궤적 포인트를 벡터에 추가
  }
  file.close(); // 파일 닫기
  bsplineSE3.feed_trajectory(trajPoints); // B-spline 궤적에 데이터 공급
}

void PointCloudProcessor::getTransformedCloud(const int i, const Eigen::Matrix4f T_criterion, pcl::PointCloud<pcl::PointXYZI> &transformed, const std::string stage)
{
  // 이미 보정된 파일을 로드
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  transformed.clear(); // 변환된 클라우드를 초기화

  // 인덱스가 타임스탬프 리스트의 범위 내인지 확인
  if (i < timestamp_lists_.size())
  {
    loadCloud(i, savePathLiDAR, "bin", *cloud); // 클라우드 로드
  }
  else
  {
    return; // 범위를 벗어나면 반환
  }

  // 기준 변환과 현재 포즈의 차이를 계산
  const auto T_diff = T_criterion.inverse() * gt_poses_.at(i);

  // 단계가 "merge"인 경우 NaN 포인트 제거
  if (stage == "merge")
  {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  }

  // 포인트 클라우드에 변환 행렬 적용
  pcl::transformPointCloud(*cloud, transformed, T_diff);   // transformed = *cloud_extrinsic;

}



