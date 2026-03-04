#============================================================
# Licensed under CC BY 4.0
# Author: YuriYuriy
# https://creativecommons.org/licenses/by/4.0/
#============================================================

import serial
import matplotlib.pyplot as plt
import math

UART_PORT  = '/dev/ttyUSB0'
UART_BPS   = 115200

TIME_BREAK = 0.001

PACKAGE_HEADER = bytes([0x55, 0xAA, 0x03, 0x08])
PACKAGE_SIZE   = 32

INTENSITY   = 10
DISTANS_MIN = 10
DISTANS_MAX = 500

POINT_MAX = 100
FRAME_MAX = 20

points_raw = []
buf_x = [0]
buf_y = [0]
frame_buf_x =[]
frame_buf_y =[]

try:
    print("\033[33mПрограмма запущена\033[0m")
    #=== Создание окна plt ===
    plt.ion()
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-DISTANS_MAX, DISTANS_MAX)
    ax.set_ylim(-DISTANS_MAX, DISTANS_MAX)
    ax.set_xlabel("X (мм)")
    ax.set_ylabel("Y (мм)")
    ax.set_title("Lidar scan")
    ax.axis('equal')
    scatter = ax.scatter([], [], s = 2)
    
    #=== Открытие порта UART ===
    ser = serial.Serial(UART_PORT, UART_BPS, timeout = 0)

    while True:
        # === Проверка закрытия окна ===
        if not plt.fignum_exists(fig.number):
            break

        #=== Чтение UART буфера ===
        data_raw = ser.read(ser.in_waiting)
        #=== Поиск заголовка пакета ===
        header_index = data_raw.find(PACKAGE_HEADER)

        #=== Поиск точек в пакете, формирование кадра ===    
        if ((header_index != -1) and (len(data_raw) >= header_index + len(PACKAGE_HEADER) + PACKAGE_SIZE)):
            package_start = header_index + len(PACKAGE_HEADER)
            package_body = data_raw[package_start:package_start + PACKAGE_SIZE]
            
            #=== Печать тела пакета === 
            #print()
            #for j in range(PACKAGE_SIZE):
            #    print(f"{j:02}", end=' ')
            #print()
            #for k in package_body:
            #    print(f"{k:02X}", end=' ')
            #print()

            #=== Определение углов === 
            angle_raw_start = package_body[2] | (package_body[3] << 8)
            angle_raw_end   = package_body[28] | (package_body[29] << 8)
                
            angle_start = (angle_raw_start / 64.0) % 360
            angle_end   = (angle_raw_end   / 64.0) % 360

            if angle_end < angle_start:
                angle_end += 360

            angle_shift = (angle_end - angle_start) / 7
            
            #=== Печать углов === 
            #print()
            #print(f"Начальный угол: {angle_start:.2f}")
            #print(f"Конечный угол:  {angle_end:.2f}")
            #print(f"Разность углов: {(angle_end - angle_start):.2f}")
            #print(f"Сдвиг:          {angle_shift:.2f}")
            
            #=== Формирование полярных координат точек ===
            for d in range(8):
                point_index = d * 3 + 4
                point_dist = package_body[point_index] | (package_body[point_index + 1] << 8)
                point_angle = angle_start + (d * angle_shift)
                point_intensity = package_body[point_index + 2]
                points_raw.append((point_dist, point_intensity, point_angle))
           
            #=== Печать полярных координат точек === 
            #print()
            #for dist, intensity, angle in points_raw:
            #    print(f"Расстояние: {dist:.2f}, Интенсивность: {intensity}, Угол: {angle:.2f}")
            
            #=== Формирование кадра === 
            for a, b, c in points_raw:
                #=== Фильрация по интенсивности отражения ===
                if ((b > INTENSITY) and (DISTANS_MIN < a < DISTANS_MAX)):
                    #=== Формирование кадра ===
                    if (len(buf_x) <= POINT_MAX):
                        x = a * math.cos(math.radians(c))
                        y = a * math.sin(math.radians(c))
                        buf_x.append(x)
                        buf_y.append(y)
                    else:
                        buf_x.clear()
                        buf_y.clear()
                        buf_x = [0]
                        buf_y = [0]                            
        points_raw.clear()

        #=== Формирование буфера кадров
        if (len(frame_buf_x) <= FRAME_MAX):
            frame_buf_x.append(buf_x.copy())
            frame_buf_y.append(buf_y.copy())
        elif (len(frame_buf_x) > FRAME_MAX):
            frame_buf_x.pop(0)
            frame_buf_y.pop(0)
        
        #=== Формирование точек ===
        all_x = [x for frame in frame_buf_x for x in frame]
        all_y = [y for frame in frame_buf_y for y in frame]
        
        #=== Отрисовка точек ===
        scatter.set_offsets(list(zip(all_x, all_y)))
        plt.pause(TIME_BREAK)

finally:
    print()
    print("\033[32mПрограмма завершена\033[0m")
    ser.close()
    plt.close()
