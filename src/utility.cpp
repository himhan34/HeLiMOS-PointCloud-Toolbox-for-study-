#include "utility.h" // utility.h 헤더 파일 포함

// 주어진 정수를 원하는 자릿수만큼 앞에 0을 추가하여 문자열로 변환하는 함수
std::string padZeros(int val, int num_digits)
{
    std::ostringstream out; // 문자열 스트림 객체 생성
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val; // 내부 정렬 및 앞에 '0' 채우기 설정
    return out.str(); // 변환된 문자열 반환
}

// 주어진 3D 점이 시야 범위(FOV)에 있는지 확인하는 함수
bool isInFOV(double x, double y, double z)
{
    // 시야각(FOV)을 라디안 단위로 변환
    const double halfFovX = 70.4 * M_PI / 180.0 / 2.0; // 방위각(x축)의 절반 FOV
    const double halfFovY = 77.2 * M_PI / 180.0 / 2.0; // 고도각(y축)의 절반 FOV

    // 데카르트 좌표계(x, y, z)를 구면 좌표계(세타, 파이)로 변환
    double theta = std::atan2(y, x);                      // 방위각 계산
    double phi = std::atan2(z, std::sqrt(x * x + y * y)); // 고도각 계산

    // FOV 내에 있는지 확인
    return std::abs(theta) <= halfFovX && phi <= halfFovY && phi >= -25 * M_PI / 180.0; // 범위 내에 있는 경우 true 반환
}

// 두 3D 점 간의 유클리드 거리 계산 함수
float euclidean_distance(Point3D p1, Point3D p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z)); // 거리 계산 및 반환
}

// 포인트 클라우드를 정규화하는 함수
void normalizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double cropSize)
{
    // 최소 및 최대 강도 값을 찾기 (포인트 클라우드는 기본적으로 최소/최대 강도 값이 없음)
    double min_intensity = std::numeric_limits<double>::max(); // 최소 강도를 가장 큰 값으로 초기화
    double max_intensity = std::numeric_limits<double>::min(); // 최대 강도를 가장 작은 값으로 초기화
    for (const auto& point : cloud->points) { // 포인트 클라우드의 각 점에 대해 반복
        if (point.intensity < min_intensity) { // 현재 강도가 최소 강도보다 작으면
            min_intensity = point.intensity; // 최소 강도를 갱신
        }
        if (point.intensity > max_intensity) { // 현재 강도가 최대 강도보다 크면
            max_intensity = point.intensity; // 최대 강도를 갱신
        }
    }

    // 점 좌표 및 강도 값을 정규화
    for (auto& point : cloud->points) { // 포인트 클라우드의 각 점에 대해 반복
        point.x = 1 / cropSize * (point.x); // x 좌표 정규화
        point.y = 1 / cropSize * (point.y); // y 좌표 정규화
        point.z = 1 / cropSize * (point.z); // z 좌표 정규화

        // 강도를 [0, 1] 범위로 정규화
        if (point.intensity > 256) { // 강도가 256보다 크면
            point.intensity = 1.0; // 강도를 1로 설정
        }
        else { // 그렇지 않으면
            point.intensity = (point.intensity - min_intensity) / (256 - min_intensity); // 강도를 정규화
        }
    }
}


// 타겟 포인트 클라우드 크기로 다운샘플링하는 함수
void downsample_to_target_size(pcl::PointCloud<pcl::PointXYZI>& cloud, size_t target_pc_size) {
    double scale_size = 1.001; // 초기 스케일 크기 설정
    pcl::PointCloud<pcl::PointXYZI> downCloud; // 다운샘플링된 포인트 클라우드를 저장할 객체

    // 입력 클라우드를 downCloud로 복사하여 다운샘플링 준비
    copyPointCloud(cloud, downCloud);
    down_sampling_voxel(downCloud, scale_size); // 다운샘플링 수행

    // 타겟 포인트 클라우드 크기에 도달할 때까지 voxel 크기 조정
    while (downCloud.points.size() < target_pc_size) {
        scale_size -= 0.002; // 스케일 크기 감소
        if (scale_size <= 0) { // 스케일 크기가 0 이하가 되면 중단
            break;
        }
        copyPointCloud(cloud, downCloud); // 원본 클라우드를 다시 복사
        down_sampling_voxel(downCloud, scale_size); // 다운샘플링 수행
    }

    // 타겟 크기를 초과하면 스케일 크기 증가로 미세 조정
    while (downCloud.points.size() > target_pc_size) {
        scale_size += 0.002; // 스케일 크기 증가
        copyPointCloud(cloud, downCloud); // 원본 클라우드를 다시 복사
        down_sampling_voxel(downCloud, scale_size); // 다운샘플링 수행
    }

    // 필요 포인트 수보다 적으면 원본 클라우드에서 랜덤 포인트 추가
    if (downCloud.points.size() < target_pc_size) {
        srand(static_cast<unsigned>(time(nullptr))); // 랜덤 숫자 생성기 시드 설정
        size_t num_extra_points = target_pc_size - downCloud.points.size(); // 추가해야 할 포인트 수 계산

        for (size_t i = 0; i < num_extra_points; ++i) {
            int idx = rand() % cloud.points.size(); // 랜덤 인덱스 선택
            downCloud.points.push_back(cloud.points[idx]); // 랜덤 포인트 추가
        }
    }

    // 다운샘플링된 클라우드를 원본 클라우드로 복사
    cloud = downCloud;
}

// 포인트 클라우드를 voxel 크기를 사용하여 다운샘플링하는 함수
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
    int intensity = rand() % 255; // 랜덤 강도 값 생성
    if (voxel_size < 0.01) { // voxel 크기가 너무 작으면 함수 종료
        return;
    }

    std::unordered_map<VOXEL_LOC, M_POINT> voxel_map; // voxel 위치와 M_POINT 객체를 매핑할 맵 생성
    uint plsize = pl_feat.size(); // 포인트 클라우드 크기 저장

    // 포인트 클라우드의 각 포인트에 대해 반복
    for (uint i = 0; i < plsize; i++) {
        pcl::PointXYZI &p_c = pl_feat[i]; // 현재 포인트 참조
        float loc_xyz[3]; // 포인트 위치를 voxel 크기로 나눈 값을 저장할 배열
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_c.data[j] / voxel_size; // 각 좌표를 voxel 크기로 나누기
            if (loc_xyz[j] < 0) { // 음수일 경우 1 감소
                loc_xyz[j] -= 1.0;
            }
        }

        // 현재 포인트의 voxel 위치를 계산
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                           (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position); // voxel 위치가 맵에 존재하는지 확인
        if (iter != voxel_map.end()) { // 존재하면 값을 누적
            iter->second.xyz[0] += p_c.x;
            iter->second.xyz[1] += p_c.y;
            iter->second.xyz[2] += p_c.z;
            iter->second.intensity += p_c.intensity;
            iter->second.count++;
        } else { // 존재하지 않으면 새로 추가
            M_POINT anp;
            anp.xyz[0] = p_c.x;
            anp.xyz[1] = p_c.y;
            anp.xyz[2] = p_c.z;
            anp.intensity = p_c.intensity;
            anp.count = 1;
            voxel_map[position] = anp; // 맵에 추가
        }
    }

    // 다운샘플링된 포인트 클라우드로 갱신
    plsize = voxel_map.size(); // 맵의 크기를 포인트 클라우드 크기로 설정
    pl_feat.clear(); // 기존 포인트 클라우드 비우기
    pl_feat.resize(plsize); // 새 크기로 조정

    uint i = 0;
    // 각 voxel 위치의 평균을 계산하여 포인트 클라우드에 추가
    for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
        pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
        pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
        pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
        pl_feat[i].intensity = iter->second.intensity / iter->second.count;
        i++;
    }
}

// 주어진 경로에 보정(calibration) 파일을 작성하는 함수
void writeCalibrationFile(const std::string &calibPath) {
    std::ofstream foutCalib(calibPath); // 보정 파일 출력 스트림 생성
    foutCalib << "P0: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl; // P0 행렬 작성
    foutCalib << "P1: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl; // P1 행렬 작성
    foutCalib << "P2: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl; // P2 행렬 작성
    foutCalib << "P3: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl; // P3 행렬 작성
    foutCalib << "Tr: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0" << std::endl; // Tr 행렬 작성
    foutCalib.close(); // 파일 닫기
}

// 1D 벡터 포즈를 4x4 변환 행렬로 변환하는 함수
void vec2tf4x4(const std::vector<float> &pose, Eigen::Matrix4f &tf4x4) {
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix(); // 쿼터니언을 회전 행렬로 변환
    tf4x4 << mat3(0, 0), mat3(0, 1), mat3(0, 2), pose[0], // 회전 및 위치 구성
             mat3(1, 0), mat3(1, 1), mat3(1, 2), pose[1],
             mat3(2, 0), mat3(2, 1), mat3(2, 2), pose[2],
             0, 0, 0, 1; // 마지막 행 설정
}

// 문자열을 구분자로 나누어 타임스탬프와 숫자 벡터로 변환하는 함수
std::pair<long long, std::vector<float>> splitLine(std::string input, char delimiter) {
    std::vector<float> answer; // 숫자 벡터 생성
    std::stringstream ss(input); // 문자열 스트림 생성
    std::string temp;
    long long timestamp; // 타임스탬프 변수
    bool check_ts = true; // 첫 번째 값을 타임스탬프로 확인하기 위한 플래그

    while (getline(ss, temp, delimiter)) { // 문자열을 구분자로 나누기
        if (check_ts) { // 첫 번째 값일 경우
            timestamp = stoll(temp); // 타임스탬프로 변환
            check_ts = false; // 플래그 해제
            continue;
        }
        answer.push_back(stof(temp)); // 나머지 값들을 float로 변환하여 벡터에 추가
    }
    return {timestamp, answer}; // 타임스탬프와 숫자 벡터 반환
}

// pcl::PointCloud를 nanoflann 포인트 클라우드 형식으로 변환하는 함수
void pcl2nanoflann(const pcl::PointCloud<pcl::PointXYZI>& src_cloud, PointCloud<num_t>& cloud) {
    int N = src_cloud.points.size(); // 포인트 개수 가져오기
    cloud.pts.resize(N); // 포인트 클라우드 크기 조정

    for (size_t i = 0; i < N; i++) { // 각 포인트에 대해 반복
        cloud.pts[i].x = src_cloud.points[i].x; // x 좌표 복사
        cloud.pts[i].y = src_cloud.points[i].y; // y 좌표 복사
        cloud.pts[i].z = src_cloud.points[i].z; // z 좌표 복사
    }
}

// 동적 대응 관계를 찾는 함수
void findDynamicCorrespondences(const pcl::PointCloud<pcl::PointXYZI> &query_cloud, 
                                const pcl::PointCloud<pcl::PointXYZI> &target_cloud, 
                                std::vector<uint32_t>& correspondences) {
    /*
    target_cloud == 병합된 클라우드 | query_cloud == A, L, O, V 클라우드
    */
    PointCloud<num_t> cloud; // nanoflann 포인트 클라우드 객체 생성
    pcl2nanoflann(target_cloud, cloud); // target_cloud를 변환

    my_kd_tree_t kdtree(3 /*차원*/, cloud, {10 /* 최대 리프 */}); // KD 트리 생성
    correspondences.resize(query_cloud.size()); // 대응 관계 벡터 크기 조정

    int query_idx = 0;
    for (auto &query_pcl : query_cloud.points) { // 각 query 포인트에 대해 반복
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z}; // query 포인트 좌표 설정
        size_t num_results = 1; // 검색 결과 개수
        std::vector<uint32_t> ret_index(num_results); // 반환 인덱스 벡터
        std::vector<num_t> out_dist_sqr(num_results); // 거리 제곱 벡터

        num_results = kdtree.knnSearch(
            &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]); // KD 트리 검색 수행

        ret_index.resize(num_results); // 결과 크기 조정
        out_dist_sqr.resize(num_results); // 거리 크기 조정

        float out_dist_sqr_score = out_dist_sqr[0]; // 거리 점수 가져오기
        if (out_dist_sqr_score < 0.03) { // 거리 점수가 기준보다 작으면
            uint32_t target_intensity = target_cloud.points[ret_index[0]].intensity; // 대상 강도 가져오기
            if (target_intensity == 0) { // 강도가 0이면
                correspondences[query_idx] = 0; // 0 설정
            } else if (target_intensity > 250) { // 강도가 250보다 크면
                correspondences[query_idx] = 251; // 251 설정
            } else if (target_intensity < 10) { // 강도가 10보다 작으면
                correspondences[query_idx] = 9; // 9 설정
            }
        } else { // 거리 점수가 기준 이상이면
            correspondences[query_idx] = 9; // 9 설정
        }
        ++query_idx; // 인덱스 증가
    }
}

// 라벨 파일을 로드하는 함수
void loadLabel(const std::string &label_name, std::vector<uint32_t> &labels) {
    std::ifstream label_input(label_name, std::ios::binary); // 바이너리 모드로 파일 열기
    if (!label_input.is_open()) { // 파일 열기에 실패한 경우
        std::cerr << "Could not open the label!" << std::endl; // 오류 메시지 출력
    }
    label_input.seekg(0, std::ios::end); // 파일 끝으로 이동
    uint32_t num_points = label_input.tellg() / sizeof(uint32_t); // 포인트 개수 계산
    label_input.seekg(0, std::ios::beg); // 파일 시작으로 이동

    labels.resize(num_points); // 라벨 벡터 크기 조정
    label_input.read((char *)&labels[0], num_points * sizeof(uint32_t)); // 라벨 데이터 읽기

    label_input.close(); // 파일 닫기
}


// 라벨을 포인트 클라우드에 할당하는 함수
void assignLabels(const std::vector<uint32_t> labels, pcl::PointCloud<pcl::PointXYZI> &cloud) {
    for (int i = 0; i < labels.size(); i++) { // 각 라벨에 대해 반복
        if (labels[i] > 250) { // 라벨이 250보다 크면
            cloud.at(i).intensity = 251; // 강도를 251로 설정
        } else if (labels[i] == 0) { // 라벨이 0이면
            cloud.at(i).intensity = 0; // 강도를 0으로 설정
        } else { // 그 외의 경우
            cloud.at(i).intensity = 9; // 강도를 9로 설정
        }
    }
}

// 라벨 데이터를 바이너리 파일로 저장하는 함수
void saveLabels(const std::string &label_name, std::vector<uint32_t> &labels) {
    std::ofstream label_output(label_name, std::ios::binary); // 바이너리 모드로 파일 열기
    if (!label_output.is_open()) { // 파일 열기에 실패한 경우
        std::cerr << "Could not open the label!" << std::endl; // 오류 메시지 출력
    }
    label_output.write(reinterpret_cast<char*>(&labels[0]), labels.size() * sizeof(uint32_t)); // 라벨 데이터 쓰기
    label_output.close(); // 파일 닫기
}


