#include "visualizer.h"

// Visualizer 클래스의 기본 생성자 정의
Visualizer::Visualizer(){};

// Visualizer 클래스의 소멸자 정의
Visualizer::~Visualizer(){};

// progressBar 함수: 현재 진행 상태를 출력하고 시간 정보를 보여주는 함수
void Visualizer::progressBar(int current, int maximum, const std::string &filename, bool saveFlag)
{
    static bool firstCall = true; // 첫 호출 여부를 저장하는 플래그

    // 상태 메시지를 설정
    std::string statusMessage;
    if (!filename.empty()) // 파일 이름이 비어 있지 않은 경우
    {
        if (!timingStarted) // 타이머가 시작되지 않은 경우
        {
            startTime = std::chrono::steady_clock::now(); // 현재 시간을 저장해 타이머 시작
            timingStarted = true; // 타이머 시작됨 표시
        }
        // 저장 중인지 처리 중인지 메시지 설정
        statusMessage = saveFlag ? "Saving: " : "Processing: ";
    }
    else
    {
        // 파일 이름이 없는 경우 "Loading the trajectory" 메시지 출력
        statusMessage = "Loading the trajectory";
    }

    // 현재 시간 계산
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = timingStarted ? now - startTime : std::chrono::seconds(0); // 경과 시간 계산

    float percentage = timingStarted ? static_cast<float>(current) / maximum : 0.0f; // 진행 퍼센트 계산
    int lpad = static_cast<int>(percentage * PBWIDTH); // 진행 막대 채움 길이 계산

    // 남은 시간과 전체 예상 시간 계산
    float remainingTime = (elapsed.count() / percentage) - elapsed.count();
    float totalTime = elapsed.count() + remainingTime;

    // 경과 시간과 전체 시간을 시, 분, 초로 변환
    int elapsedHours = static_cast<int>(elapsed.count()) / 3600;
    int elapsedMinutes = (static_cast<int>(elapsed.count()) / 60) % 60;
    int elapsedSeconds = static_cast<int>(elapsed.count()) % 60;

    int totalHours = static_cast<int>(totalTime) / 3600;
    int totalMinutes = (static_cast<int>(totalTime) / 60) % 60;
    int totalSeconds = static_cast<int>(totalTime) % 60;

    if (!firstCall) // 첫 호출이 아닌 경우
    {
        // 커서를 위로 두 줄 이동해 이전 출력을 덮어쓰기
        printf("\033[2A");
    }

    // 현재 줄을 지우고 커서를 맨 앞으로 이동
    printf("\r\033[K");

    // 첫 줄에 상태 메시지 출력
    std::cout << cyan << statusMessage << filename << reset << "\n";

    // 다음 줄을 지우고 커서를 맨 앞으로 이동
    printf("\r\033[K");

    // 진행 막대와 시간 정보를 두 번째 줄에 출력
    if (timingStarted) // 타이머가 시작된 경우
    {
        float binsPerSec = static_cast<float>(current / elapsed.count()); // 초당 처리된 bin 수 계산
        // 진행 퍼센트, 막대, 경과 시간, 예상 완료 시간, 처리 속도 출력
        std::cout << std::fixed << std::setprecision(2) << percentage * 100 << "% ["
                  << std::string(lpad, '=') << std::string(PBWIDTH - lpad, ' ')
                  << "] " << current << "/" << maximum
                  << " [" << std::setfill('0') << std::setw(2) << elapsedHours << ":"
                  << std::setw(2) << elapsedMinutes << ":" << std::setw(2) << elapsedSeconds
                  << "<" << std::setw(2) << totalHours << ":" << std::setw(2) << totalMinutes
                  << ":" << std::setw(2) << totalSeconds << ", " << binsPerSec << "bin/s]"
                  << "\n";
    }
    else // 타이머가 시작되지 않은 경우
    {
        // 진행 상태를 표시할 수 없는 경우 기본 메시지 출력
        std::cout << "--% [" << std::string(PBWIDTH, ' ') << "] "
                  << "--/-- [--:--:--<--:--:--, --bin/s]"
                  << "\n";
    }

    firstCall = false; // 첫 호출 플래그 해제
    fflush(stdout); // 출력 버퍼 비우기
}

// 사용자 입력을 받아오는 함수
std::string Visualizer::getInput(const std::string &prompt)
{
    std::cout << prompt; // 프롬프트 메시지 출력
    std::string input; // 사용자 입력을 저장할 문자열
    std::getline(std::cin, input); // 입력 받기
    return input; // 입력 반환
}

// LiDAR 타입을 문자열로 변환하는 함수
std::string Visualizer::lidarTypeToString(LiDARType lidarType)
{
    // LiDAR 타입에 따라 문자열 반환
    switch (lidarType)
    {
    case OUSTER:
        return "Ouster";
    case VELODYNE:
        return "Velodyne";
    case LIVOX:
        return "Livox";
    case AEVA:
        return "Aeva";
    default:
        return "Unknown"; // 알 수 없는 타입인 경우
    }
}
