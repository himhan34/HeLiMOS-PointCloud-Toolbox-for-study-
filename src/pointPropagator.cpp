#include "PointCloudProcessor.h" // PointCloudProcessor 클래스 포함

int main(int argc, char **argv)
{
    // YAML 파일을 로드하여 설정 객체 생성
    YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");

    // 각 포인트 클라우드 프로세서를 설정과 인덱스로 초기화하고 경로와 자세 데이터를 로드
    PointCloudProcessor OusterProcessor(config, 0);
    OusterProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor VelodyneProcessor(config, 1);
    VelodyneProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor AviaProcessor(config, 2);
    AviaProcessor.loadTrajectoryAndPoses();

    PointCloudProcessor AevaProcessor(config, 3);
    AevaProcessor.loadTrajectoryAndPoses();

    // 각 포인트 클라우드 프로세서의 타임스탬프 벡터 참조
    const auto &ts_o = OusterProcessor.timestamp_lists_;
    const auto &ts_v = VelodyneProcessor.timestamp_lists_;
    const auto &ts_a = AviaProcessor.timestamp_lists_;
    const auto &ts_e = AevaProcessor.timestamp_lists_;

    // 첫 번째 타임스탬프 출력
    std::cout << std::fixed << OusterProcessor.timestamp_lists_[0] << std::endl;
    std::cout << VelodyneProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AviaProcessor.timestamp_lists_[0] << std::endl;
    std::cout << AevaProcessor.timestamp_lists_[0] << std::endl;

    // 두 타임스탬프가 지정된 시간 차이 내에 있는지 확인하는 람다 함수
    auto isDifferenceWithin = [](long long a, long long b, double diff_in_sec) {
        auto diff = static_cast<double>(std::abs(a - b)) / 1e9; // 나노초를 초로 변환
        return std::abs(diff) <= diff_in_sec; // 차이가 지정된 시간 내에 있는지 반환
    };

    // 타임스탬프 벡터의 최소 및 최대 크기를 찾는 람다 함수
    auto findMinMaxSize = [](const std::vector<long long> &ts_o,
                             const std::vector<long long> &ts_v,
                             const std::vector<long long> &ts_a,
                             const std::vector<long long> &ts_e) {
        size_t minSize = std::min({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()}); // 최소 크기 계산
        size_t maxSize = std::max({ts_o.size(), ts_v.size(), ts_a.size(), ts_e.size()}); // 최대 크기 계산
        return std::make_tuple(minSize, maxSize); // 최소 및 최대 크기 반환
    };

    // Ouster의 첫 번째 타임스탬프가 가장 먼저 오는지 확인
    if (ts_o[0] < ts_v[0] && ts_o[0] < ts_a[0] && ts_o[0] < ts_e[0]) {
        std::cout << std::fixed << "Check Ouster comes first!" << std::endl;
    } else {
        throw std::runtime_error("Currently, only KAIST05 is supported while strongly assuming that the Ouster comes first");
    }

    double diff_in_sec = 0.08; // 허용되는 시간 차이

    const auto &[minSize, maxSize] = findMinMaxSize(ts_o, ts_v, ts_a, ts_e); // 최소 및 최대 크기 가져오기

    // 최소 크기 내에서 모든 타임스탬프가 일치하는지 확인
    for (int i = 0; i < minSize; ++i) {
        if (isDifferenceWithin(ts_o[i], ts_v[i], diff_in_sec) && isDifferenceWithin(ts_o[i], ts_a[i], diff_in_sec)
            && isDifferenceWithin(ts_o[i], ts_e[i], diff_in_sec)) {
            continue; // 일치하면 다음 반복으로
        } else {
            throw std::runtime_error("Something's wrong. Timestamp is not matched :(");
        }
    }

    // 병합된 파일 저장 경로 설정
    std::string mergedSavePath = config["Path"]["savePath"].as<std::string>() + "Merged/";
    std::string mergedSavePathLiDAR = mergedSavePath + "velodyne/";
    std::string mergedSavePathLabel = mergedSavePath + "labels/";
    std::string mergedSavePathPose = mergedSavePath + "poses.txt";
    std::string mergedSavePathCalib = mergedSavePath + "calib.txt";

    // 최대 크기 내에서 각 프레임에 대해 처리
    for (int i = 0; i < maxSize; ++i) {
        // 병합된 라벨 파일 이름 생성
        std::string mergedLabelName = mergedSavePathLabel + padZeros(i, 6) + ".label";
        if (!std::filesystem::exists(mergedLabelName)) { // 파일이 존재하지 않으면
            continue; // 다음 반복으로
        }
        std::cout << green << "Processing " << i << "th frame" << reset << std::endl;

        // 포인트 클라우드 및 라벨 벡터 초기화
        pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr aevaCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr aviaCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ousterCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr veloCloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        std::vector<uint32_t> mergedLabel;
        std::vector<uint32_t> aevaLabel;
        std::vector<uint32_t> aviaLabel;
        std::vector<uint32_t> ousterLabel;
        std::vector<uint32_t> veloLabel;

        // 1. 각 포인트 클라우드를 Ouster 프레임으로 변환
        const Eigen::Matrix4f T_o = OusterProcessor.gt_poses_.at(i);

        // 2-1. 라벨된 병합된 포인트 클라우드 로드
        loadCloud(i, mergedSavePathLiDAR, "bin", *mergedCloud);
        loadLabel(mergedLabelName, mergedLabel);
        assignLabels(mergedLabel, *mergedCloud);

        // 2-2. Ouster로 전파
        loadCloud(i, OusterProcessor.savePathLiDAR, "bin", *ousterCloud);
        findDynamicCorrespondences(*ousterCloud, *mergedCloud, ousterLabel);
        std::string saveOusterLabelPath = OusterProcessor.savePathLabel + padZeros(i, 6) + ".label";
        saveLabels(saveOusterLabelPath, ousterLabel);
        std::cout << "Saved " << i << "th propagated Ouster label" << std::endl;

        // 2-3. Velodyne로 전파
        VelodyneProcessor.getTransformedCloud(i, T_o, *veloCloud, "propagated");
        findDynamicCorrespondences(*veloCloud, *mergedCloud, veloLabel);
        std::string saveVeloLabelPath = VelodyneProcessor.savePathLabel + padZeros(i, 6) + ".label";
        saveLabels(saveVeloLabelPath, veloLabel);
        std::cout << "Saved " << i << "th propagated Velodyne label" << std::endl;

        // 2-4. Avia로 전파
        AviaProcessor.getTransformedCloud(i, T_o, *aviaCloud, "propagated");
        findDynamicCorrespondences(*aviaCloud, *mergedCloud, aviaLabel);
        std::string saveAviaLabelPath = AviaProcessor.savePathLabel + padZeros(i, 6) + ".label";
        saveLabels(saveAviaLabelPath, aviaLabel);
        std::cout << "Saved " << i << "th propagated Avia label" << std::endl;

        // 2-5. Aeva로 전파
        AevaProcessor.getTransformedCloud(i, T_o, *aevaCloud, "propagated");
        findDynamicCorrespondences(*aevaCloud, *mergedCloud, aevaLabel);
        std::string saveAevaLabelPath = AevaProcessor.savePathLabel + padZeros(i, 6) + ".label";
        saveLabels(saveAevaLabelPath, aevaLabel);
        std::cout << "Saved " << i << "th propagated Aeva label" << std::endl;
    }
    return 0; // 프로그램 종료
}
