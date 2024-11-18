#include "PointCloudProcessor.h" // PointCloudProcessor 클래스 포함

int main(int argc, char **argv)
{
    // YAML 파일을 로드하여 설정 객체 생성
    YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");

    // Ouster 포인트 클라우드 프로세서 초기화 및 처리
    PointCloudProcessor OusterProcessor(config, 0); // 설정과 인덱스 0으로 초기화
    OusterProcessor.loadTrajectoryAndPoses(); // 경로와 자세 데이터 로드
    OusterProcessor.loadAndProcessBinFiles(); // 바이너리 파일 로드 및 처리

    // Velodyne 포인트 클라우드 프로세서 초기화 및 처리
    PointCloudProcessor VelodyneProcessor(config, 1); // 설정과 인덱스 1로 초기화
    VelodyneProcessor.loadTrajectoryAndPoses(); // 경로와 자세 데이터 로드
    VelodyneProcessor.loadAndProcessBinFiles(); // 바이너리 파일 로드 및 처리

    // Avia 포인트 클라우드 프로세서 초기화 및 처리
    PointCloudProcessor AviaProcessor(config, 2); // 설정과 인덱스 2로 초기화
    AviaProcessor.loadTrajectoryAndPoses(); // 경로와 자세 데이터 로드
    AviaProcessor.loadAndProcessBinFiles(); // 바이너리 파일 로드 및 처리

    // Aeva 포인트 클라우드 프로세서 초기화 및 처리
    PointCloudProcessor AevaProcessor(config, 3); // 설정과 인덱스 3으로 초기화
    AevaProcessor.loadTrajectoryAndPoses(); // 경로와 자세 데이터 로드
    AevaProcessor.loadAndProcessBinFiles(); // 바이너리 파일 로드 및 처리

    return 0; // 프로그램 종료
}
