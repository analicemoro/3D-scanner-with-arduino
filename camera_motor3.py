import pyrealsense2 as rs
import numpy as np
import cv2
import os
import serial
import time
import datetime

pc_path = "C:/Users/Analice/.vscode/saved_images" + datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
os.makedirs(pc_path)

play_playback = True


def deproject(depth_intrinsics, depth_frame, x, y):
    vdist = depth_frame.get_distance(x, y)
    point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], vdist)
    return point


def image_to_pointcloud(depth_frame, depth_intrinsics, color_image):
    # Assumindo que color_image é uma imagem RGB e depth_frame é o frame de profundidade
    height, width = color_image.shape[:2]

    # Inicializa uma lista para armazenar os pontos 3D
    points_3d = []

    # Itera sobre cada pixel na imagem
    for y in range(height):
        for x in range(width):
            # Deprojetar o pixel para obter a coordenada 3D
            point_3d = deproject(depth_intrinsics, depth_frame, x, y)
            if not np.isnan(point_3d).any():
                # Adiciona o ponto 3D e a cor correspondente à lista
                points_3d.append((point_3d, color_image[y, x]))

    return np.array(points_3d)


def send_command(ser, speed, steps):
    command = f"{speed},{steps}\n"
    ser.write(command.encode())


# Configuração da porta serial
arduino_port = "COM6"  # número da porta
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

while play_playback:
    # configuração da câmera
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    # pipeline_profile = config.resolve(pipeline_wrapper)
    # device = pipeline_profile.get_device()
    # device_product_line = str(device.get_info(rs.camera_info.product_line))

    try:

        # Inicia o streaming
        profile = pipeline.start(config)

        device = profile.get_device()
        if not device:
            print("No device")
            break

        playback = device.as_playback()
        if playback:
            playback.set_real_time(False)

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("A demonstração requer uma câmera de profundidade com sensor de cor")
            exit(0)

        # Obtém escala de profundidade do sensor
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("A escala de profundidade é:", depth_scale)

        # Removeremos o fundo dos objetos mais distantes que clipping_distance_in_meters
        clipping_distance_in_meters = 1  # 1 metro
        clipping_distance = clipping_distance_in_meters / depth_scale

        # Cria um objeto de alinhamento rs.align nos permite realizar o alinhamento de frames de profundidade para outros frames
        align_to = rs.stream.color
        align = rs.align(align_to)

        # frame_count = 0
        time.sleep(2)  # Aguarda o Arduino reiniciar
        # while True:
        for frame_count in range(0, 35):
            # Obtém o conjunto de frames de cor e profundidade
            frames = pipeline.wait_for_frames()
            if playback:
                playback.pause()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Alinha o frame de profundidade ao frame de cor
            aligned_frames = align.process(frames)

            # Obtém frames alinhados
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            # Valida se ambos os frames são válidos
            if not aligned_depth_frame or not aligned_color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            height, width = depth_image.shape
            x, y = np.meshgrid(np.arange(width), np.arange(height))
            points_3d = np.stack((x, y, depth_image), axis=-1)

            depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            # get pointcloud with get_vertices
            pc = rs.pointcloud()
            pc.map_to(aligned_color_frame)
            points = pc.calculate(aligned_depth_frame)
            vtx = np.asanyarray(points.get_vertices())
            tex = np.asanyarray(points.get_texture_coordinates())

            vtx_list = vtx.tolist()
            # data_list.append(vtx_list)
            point_cloud_in_numpy = np.asarray(vtx_list)

            point_cloud_valid_only = np.zeros_like(point_cloud_in_numpy)

            # 3D filter for pointcloud

            radius = 0.5

            pointcloud_X = point_cloud_in_numpy[:, 0]
            pointcloud_Y = point_cloud_in_numpy[:, 1]
            pointcloud_Z = point_cloud_in_numpy[:, 2]
            pointcloud_mask = 2 * (pointcloud_X ** 2) + 2 * (pointcloud_Y ** 2) + 2 * (pointcloud_Z ** 2) <= 3 * (
                            radius ** 2)

            data = point_cloud_in_numpy[pointcloud_mask]
            path_save = pc_path + "/{:04d}.npy".format(frame_count)

            # Remove fundo - Define pixels além de clipping_distance para cinza
            # grey_color = 153
            # depth_image_3d = np.dstack(
            #     (depth_image, depth_image, depth_image))  # a imagem de profundidade é 1 canal, cor é 3 canais
            # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # Renderiza as imagens:
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # images = np.hstack((pointcloud_mask, depth_colormap))

            cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
            cv2.imshow('Depth', depth_colormap)
            key = cv2.waitKey(1)

            # Pressione 's' para salvar as imagens atuais
            # if key & 0xFF == ord('s'):
            cv2.imwrite(os.path.join(pc_path, f"color_image_{frame_count}.png"), color_image)
            np.save(os.path.join(pc_path, f"depth_array_{frame_count}.npy"), depth_image)
            print(f"Imagens salvas como 'color_image_{frame_count}.png' e 'depth_array_{frame_count}.npy'.")

            pointcloud = image_to_pointcloud(aligned_depth_frame, depth_intrinsics, color_image)
            np.save(os.path.join(pc_path, f'pointcloud_deproject_{frame_count}.npy'), pointcloud)
            print(f"Point cloud saved to 'pointcloud_deproject_{frame_count}.npy' with {len(pointcloud)} points.")
            # frame_count += 1

            # Envia comandos para o Arduino
            speed = 20
            steps = 20
            send_command(ser, speed, steps)
            time.sleep(1)  # Pequeno delay para evitar sobrecarga de comandos

            if playback:
                playback.resume()

            # Pressione esc ou 'q' para fechar a janela de imagem
            # if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

    finally:
        pipeline.stop()
        ser.close()
