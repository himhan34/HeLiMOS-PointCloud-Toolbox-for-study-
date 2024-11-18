import pandas as pd  # pandas 라이브러리를 가져옵니다.
import numpy as np  # numpy 라이브러리를 가져옵니다.
import open3d as o3d  # Open3D 라이브러리를 가져옵니다.
from scipy.spatial.transform import Rotation as R  # 회전 변환을 위해 scipy 라이브러리의 Rotation 클래스를 가져옵니다.
from pyproj import Proj, transform  # 좌표 변환을 위해 pyproj 라이브러리의 Proj와 transform 함수를 가져옵니다.

# 위도, 경도, 고도를 로컬 좌표계로 변환하는 함수입니다.
def latlon2local(lat, lon, alt, origin):
    # 위도/경도를 위한 투영 정의 (WGS84 기준)
    wgs84 = Proj(proj='latlong', datum='WGS84')

    # 로컬 좌표계 투영 정의: 동쪽, 북쪽, 위쪽 (origin이 기준점)
    local = Proj(proj='aeqd', lat_0=origin[0], lon_0=origin[1], datum='WGS84')

    # 지리 좌표를 로컬 좌표로 변환합니다.
    xEast, yNorth = transform(wgs84, local, lon, lat)

    # 고도의 경우, 기준점의 고도를 뺀 값을 사용합니다.
    zUp = alt - origin[2]

    return xEast, yNorth, zUp

# 경로 데이터를 처리하는 함수입니다.
def processTrajectory(inputPath, outputPath, typeLiDAR):
    # CSV 파일을 읽어옵니다.
    latlonheight = pd.read_csv(inputPath, header=None)
    latlonheight = latlonheight.iloc[:, :10].to_numpy()  # 데이터를 numpy 배열로 변환합니다.

    # 위도, 경도, 고도를 로컬 좌표로 변환합니다.
    x, y, z = latlon2local(latlonheight[:, 1], latlonheight[:, 2], latlonheight[:, 3], [latlonheight[0, 1], latlonheight[0, 2], latlonheight[0, 3]])

    # 왼손 좌표계 각도를 오른손 좌표계로 변환합니다.
    rpy = latlonheight[:, 7:10]
    rpy[:, 2] = -rpy[:, 2]  # Yaw 값을 반대로 바꿉니다.
    rpy = np.roll(rpy, 1, axis=1)  # Roll, Pitch, Yaw 순서를 변경합니다.
    rpyRad = np.deg2rad(rpy)  # 각도를 라디안 단위로 변환합니다.

    # Roll, Pitch, Yaw를 쿼터니언으로 변환합니다.
    # 'ZYX' 순서 (Yaw-Pitch-Roll)를 따릅니다.
    quat = R.from_euler('zyx', rpyRad).as_quat()

    # 쿼터니언 데이터를 포함한 새로운 배열을 생성합니다.
    latlonheight = np.column_stack((latlonheight[:, 0], x, y, z, quat))

    # LiDAR 변환 행렬을 정의합니다 (R_IL, R_IA, R_IO, R_IV)
    R_IL = np.array([[0.999260281307309, 0.0383884500761334, 0.00228409771914936, 0.205400000000000],
                     [-0.0383981303207532, 0.999253021123844, 0.00435699010113986, -0.352800000000000],
                     [-0.00211513344942227, -0.00444147223600340, 0.999987899694225, 0.0706000000000000],
                     [0, 0, 0, 1]])
    R_IA = np.array([[0.998593404866819, 0.0471428768856460, -0.0242643960451897, 0.133700000000000],
                     [-0.0476188752386917, 0.998676624578879, -0.0194278727795602, 0.355700000000000],
                     [0.0233163993252583, 0.0205559888762785, 0.999516781671935, 0.221900000000000],
                     [0, 0, 0, 1]])  # R_IA 행렬입니다.

    R_IO = np.array([[0.999715495593027, 0.0223448061210468, -0.00834490926264448, -0.417000000000000],
                     [-0.0224514077723064, 0.999664614804883, -0.0129070599303583, -0.00300000000000000],
                     [0.00805370475188661, 0.0130907427756056, 0.999881878170293, 0.299600000000000],
                     [0, 0, 0, 1]])  # R_IO 행렬입니다.

    R_IV = np.array([[0.999995760375673, 0.000773891425480788, -0.00280719125478291, -1.37200000000000],
                     [-0.000825966811080870, 0.999826715354752, -0.0185972320992637, 0.0668000000000000],
                     [0.00279231257318289, 0.0185994719007949, 0.999823115673720, 0.0297000000000000],
                     [0, 0, 0, 1]])  # R_IV 행렬입니다.

    # 적절한 LiDAR 행렬을 선택합니다.
    if typeLiDAR == "Livox":
        LiDAR = R_IL
    elif typeLiDAR == "Aeva":
        LiDAR = R_IA
    elif typeLiDAR == "Ouster":
        LiDAR = R_IO
    elif typeLiDAR == "Velodyne":
        LiDAR = R_IV
    else:
        raise ValueError("Invalid LiDAR type")  # 유효하지 않은 LiDAR 타입일 경우 오류를 발생시킵니다.

    # R_IN 변환 행렬입니다.
    R_IN = np.array([[0.0110181061714567, -0.999760525916693, -0.0189213279101856, -0.0136388586688681],
                     [0.999915259583709, 0.0108843018002586, 0.00715997514944445, 0.163678520730871],
                     [-0.00695231049069956, -0.0189986948218794, 0.999795674892162, -0.00743152306072198],
                     [0, 0, 0, 1]])

    # 변환된 경로 데이터를 저장합니다.
    trajLiDAR = latlonheight
    meshArrayTrajLiDAR = []

    # 변환된 경로 데이터에 대해 각 포즈를 처리합니다.
    for i in range(len(trajLiDAR)):
        # 쿼터니언을 선택합니다 (qx, qy, qz, qw)
        quat = trajLiDAR[i, 4:8]
        rot = R.from_quat(quat).as_matrix()  # 쿼터니언을 회전 행렬로 변환합니다.

        # 변환을 적용합니다.
        trajLiDAR[i, 1:4] += (rot @ (R_IN[:3, :3] @ LiDAR[:3, 3] + R_IN[:3, 3])).T
        rotLiDAR = rot @ R_IN[:3, :3] @ LiDAR[:3, :3]  # 회전 변환을 적용합니다.
        trajLiDAR[i, 4:8] = R.from_matrix(rotLiDAR).as_quat()  # 회전 행렬을 쿼터니언으로 변환합니다.

        # 각 20번째 포즈마다 좌표축 프레임을 추가합니다.
        if(i % 20 == 0):
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2)  # 좌표축 프레임을 생성합니다.
            pose = np.identity(4)  # 기본 변환 행렬을 생성합니다.
            pose[:3, 3] = trajLiDAR[i, 1:4]  # 위치를 설정합니다.
            pose[:3, :3] = rotLiDAR  # 회전을 설정합니다.
            meshArrayTrajLiDAR.append(mesh.transform(pose))  # 변환된 메쉬를 추가합니다.

    # 결과를 파일로 저장합니다.
    fmt = ['%.0f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f', '%.20f']  # 포맷을 정의합니다.
    np.savetxt(outputPath, trajLiDAR, fmt=fmt, delimiter=' ')  # 데이터를 텍스트 파일로 저장합니다.
    
    print("Saved to " + outputPath)  # 저장 완료 메시지를 출력합니다.
    print("Visualizing the trajectory...")  # 시각화 메시지를 출력합니다.
    print("Press 'q' to exit")  # 'q'를 눌러 종료하라는 안내 메시지를 출력합니다.
    o3d.visualization.draw_geometries(meshArrayTrajLiDAR)  # 경로를 시각화합니다.
    print("Done")  # 완료 메시지를 출력합니다.

if __name__ == "__main__":
    # 사용자 입력을 받습니다.
    inputCSV = input("Enter the path of the input CSV file: ")  # 입력 CSV 파일 경로를 받습니다.
    outputTXT = input("Enter the path of the output file: ")  # 출력 파일 경로를 받습니다.
    typeLiDAR = input("Enter the LiDAR type (Livox, Aeva, Ouster, Velodyne): ")  # LiDAR 타입을 받습니다.

    processTrajectory(inputCSV, outputTXT, typeLiDAR)  # 경로 데이터를 처리합니다.
