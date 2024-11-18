```python
import open3d as o3d  # Open3D 라이브러리를 가져옵니다.
import numpy as np  # Numpy 라이브러리를 가져옵니다.
import struct  # 바이너리 데이터를 읽기 위해 struct 모듈을 가져옵니다.
import os  # 파일 경로 처리를 위해 os 모듈을 가져옵니다.

# 바이너리 파일을 읽는 함수입니다.
def read_bin_file(filename, typeLiDAR):
    points = []  # 포인트 클라우드를 저장할 리스트입니다.
    with open(filename, "rb") as file:  # 바이너리 모드로 파일을 엽니다.
        while True:  # 파일의 끝까지 데이터를 읽습니다.
            # LiDAR 타입이 "Velodyne"인 경우
            if typeLiDAR == "Velodyne":
                data = file.read(22)  # 22바이트를 읽습니다.
                if len(data) < 22:  # 데이터가 충분하지 않으면 반복을 종료합니다.
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])  # 좌표와 강도 값을 해석합니다.
                ring = struct.unpack('H', data[16:18])  # 링 번호를 해석합니다.
                time = struct.unpack('f', data[18:22])  # 시간 정보를 해석합니다.
            # LiDAR 타입이 "Ouster"인 경우
            elif typeLiDAR == "Ouster":
                data = file.read(26)  # 26바이트를 읽습니다.
                if len(data) < 26:  # 데이터가 충분하지 않으면 반복을 종료합니다.
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])  # 좌표와 강도 값을 해석합니다.
                t = struct.unpack('I', data[16:20])  # 타임스탬프를 해석합니다.
                reflectivity, ring, ambient = struct.unpack('HHH', data[20:26])  # 반사율, 링 번호, 주변 광량을 해석합니다.
            # LiDAR 타입이 "Aeva"인 경우 (특정 조건에 따라 나뉨)
            elif typeLiDAR == "Aeva" and int(filename.split("/")[-1].split('.')[0]) > 1691936557946849179: 
                data = file.read(29)  # 29바이트를 읽습니다.
                if len(data) < 29:  # 데이터가 충분하지 않으면 반복을 종료합니다.
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])  # 좌표, 반사율, 속도를 해석합니다.
                time_offset_ns = struct.unpack('I', data[20:24])  # 시간 오프셋을 해석합니다.
                line_index = struct.unpack('B', data[24:25])  # 라인 인덱스를 해석합니다.
                intensity = struct.unpack('f', data[25:29])  # 강도 값을 해석합니다.
            # LiDAR 타입이 "Aeva"인 경우 (다른 조건)
            elif typeLiDAR == "Aeva" and int(filename.split("/")[-1].split('.')[0]) <= 1691936557946849179: 
                data = file.read(25)  # 25바이트를 읽습니다.
                if len(data) < 25:  # 데이터가 충분하지 않으면 반복을 종료합니다.
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])  # 좌표, 반사율, 속도를 해석합니다.
                time_offset_ns = struct.unpack('I', data[20:24])  # 시간 오프셋을 해석합니다.
                line_index = struct.unpack('B', data[24:25])  # 라인 인덱스를 해석합니다.
            # LiDAR 타입이 "Livox"인 경우
            elif typeLiDAR == "Livox":
                data = file.read(19)  # 19바이트를 읽습니다.
                if len(data) < 19:  # 데이터가 충분하지 않으면 반복을 종료합니다.
                    break
                x, y, z = struct.unpack('fff', data[:12])  # 좌표 값을 해석합니다.
                reflectivity, tag, line = struct.unpack('BBB', data[12:15])  # 반사율, 태그, 라인 정보를 해석합니다.
                offset_time = struct.unpack('f', data[15:19])  # 오프셋 시간을 해석합니다.
            else:  # 지원되지 않는 LiDAR 타입인 경우
                raise ValueError("Unsupported LiDAR type")  # 오류를 발생시킵니다.
            points.append([x, y, z])  # 포인트 클라우드 리스트에 좌표를 추가합니다.
    return points  # 포인트 클라우드를 반환합니다.

# 포인트 클라우드를 시각화하는 함수입니다.
def visualize_point_cloud(points):
    pcd = o3d.geometry.PointCloud()  # PointCloud 객체를 생성합니다.
    pcd.points = o3d.utility.Vector3dVector(np.array(points))  # 포인트 클라우드를 설정합니다.
    o3d.visualization.draw_geometries([pcd])  # 포인트 클라우드를 시각화합니다.

# 예제 실행
filename = input("Enter the path of bin file: ")  # 파일 경로를 입력받습니다.
typeLiDAR = input("Enter the LiDAR type (Livox, Aeva, Ouster, Velodyne): ")  # LiDAR 타입을 입력받습니다.
pointcloud = read_bin_file(filename, typeLiDAR)  # 포인트 클라우드를 읽습니다.
visualize_point_cloud(pointcloud)  # 포인트 클라우드를 시각화합니다.
