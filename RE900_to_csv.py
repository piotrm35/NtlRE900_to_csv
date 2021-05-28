"""
/***************************************************************************
  NtlRE900_to_csv.py

  A script that get DVR data (created with "Navitel RE900") and produces csv file and jpg files for QGIS with "Road Inspection Viewer" plugin.
  This script uses opencv library.
  --------------------------------------
  version : 0.6
  Date : 27-05-2021
  Copyright: (C) 2021 by Piotr Michałowski
  Email: piotrm35@hotmail.com
/***************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as published
 * by the Free Software Foundation.
 *
 ***************************************************************************/
"""
#-----------------------------------------------------------------------------------------------------------------------------------------
# installation of libraries (Ubuntu):


# sudo apt-get install python3-opencv
# python3 -c "import cv2; print(cv2.__version__)"    # my result: 'v. 4.2.0+dfsg-5'


#=========================================================================================================================================
# SETUP:

INPUT_DVR_FOLDER_PATH = 'Navitel DVR test'
OUTPUT_PIC_FOLDER_PATH = 'out'
OUTPUT_CSV_FILE_NAME = 'out.csv'

NMEA_FILES_EXTENSION = '.nmea'
VIDEO_FILES_EXTENSION = '.mp4'

MIN_DISTANCE_BETWEEN_PICTURES_m = 4

#=========================================================================================================================================
# globals:

import os, time, datetime
import math
import glob
import cv2


HEADER_TUPLE = ('lat', 'lon', 'time_stamp', 'file_names')
cos_p0 = None

#-----------------------------------------------------------------------------------------------------------------------------------------
# work functions:

# from https://en.wikipedia.org/wiki/NMEA_0183
'''
calculate NMEA string CRC
IN: NMEA string between $ and *, ex '$GPGGA,154548.000,5346.7200,N,02030.3958,E,1,8,0.96,115.0,M,35.2,M,,*54' would be 'GPGGA,154548.000,5346.7200,N,02030.3958,E,1,8,0.96,115.0,M,35.2,M,,'
OUT: checksum to place after *
'''
def calculateNMEAchecksum(NMEAstring):
    calc_cksum = 0
    for s in NMEAstring:
        #it is XOR of ech Unicode integer representation
        calc_cksum ^= ord(s)
    calc_cksum = hex(calc_cksum) #get hex representation
    calc_cksum = f'{calc_cksum}'[2:] #cut 0x
    return calc_cksum

def check_mnea_data_line_CRC(mnea_data_line):
    try:
        asterisk_list = mnea_data_line.split('*')
        if len(asterisk_list) == 2:
            crc_1 = asterisk_list[1].upper()
            crc_2 = calculateNMEAchecksum(asterisk_list[0][1:]).upper()
            return crc_1 == crc_2
    except:
        pass
    return False

def get_data_list(mnea_data_line):
    try:
        comma_list = mnea_data_line.split(',')
        speed_in_knots = ''
        if mnea_data_line.startswith('$GPGGA'):
            quality_of_measurement  = int(comma_list[6])
            if quality_of_measurement <= 0 or quality_of_measurement >= 6:
                return None
            data_timeliness_UTC = comma_list[1]
            lat_lon_list = comma_list[2:6]
        elif mnea_data_line.startswith('$GPRMC'):
            quality_of_measurement  = comma_list[2].upper()
            speed_in_knots  = comma_list[7]
            if quality_of_measurement != 'A':
                return None
            data_timeliness_UTC = comma_list[1]
            lat_lon_list = comma_list[3:7]
        else:
            return None
        lat = lat_lon_list[0]
        lat = float(lat[0:2]) + float(lat[2:]) / 60
        if lat_lon_list[1].upper() == 'N':
            pass
        elif lat_lon_list[1].upper() == 'S':
            lat *= -1
        else:
            return None
        lon = lat_lon_list[2]
        lon = float(lon[0:3]) + float(lon[3:]) / 60
        if lat_lon_list[3].upper() == 'E':
            pass
        elif lat_lon_list[3].upper() == 'W':
            lon *= -1
        else:
            return None
        return [data_timeliness_UTC, lat, lon, speed_in_knots]
    except:
        pass
    return None

def get_cos_p0(lat):
    global cos_p0
    if cos_p0 is None:
        cos_p0 = math.cos(lat * math.pi / 180.0)
    return cos_p0

def calculate_distance_by_BL_WGS_84(lat_1, lon_1, lat_2, lon_2):    # in meters
    delta_lat_in_km = (lat_2 - lat_1) * 111.197
    delta_lon_in_km = (lon_2 - lon_1) * 111.197 * get_cos_p0(lat_1)
    return 1000 * math.sqrt(delta_lat_in_km * delta_lat_in_km + delta_lon_in_km * delta_lon_in_km)

def calculate_position_of_picture(nav_data_dict, pic_time_shift_float):     # time_shift - elapsed time, in seconds, from the beginning of the video file
    pos_time_shift_1_str = None
    pos_time_shift_2_str = None
    prev_time_shift_str = None
    pos_speed_str = None
    prev_speed_str = None
    for pos_time_shift_str in nav_data_dict.keys():
        if pos_time_shift_1_str is None and float(pos_time_shift_str) > pic_time_shift_float:
            pos_time_shift_1_str = prev_time_shift_str
            pos_time_shift_2_str = pos_time_shift_str
            pos_speed_str = nav_data_dict[pos_time_shift_str][2]
            break
        prev_time_shift_str = pos_time_shift_str
        prev_speed_str = nav_data_dict[pos_time_shift_str][2]
    if pos_time_shift_1_str is None or pos_time_shift_2_str is None:
        return None
    if pos_speed_str and (len(pos_speed_str) == 0 or float(pos_speed_str) > 0):
        pos_speed_condition = True
    else:
        pos_speed_condition = False
    if prev_speed_str and (len(prev_speed_str) == 0 or float(prev_speed_str) > 0):
        prev_speed_condition = True
    else:
        prev_speed_condition = False
    if not pos_speed_condition or not prev_speed_condition:
        return None
    pos_time_shift_1_float = float(pos_time_shift_1_str)
    pos_time_shift_2_float = float(pos_time_shift_2_str)
    dividing_coefficient = (pic_time_shift_float - pos_time_shift_1_float) / (pos_time_shift_2_float - pos_time_shift_1_float)
    delta_lat = (nav_data_dict[pos_time_shift_2_str][0] - nav_data_dict[pos_time_shift_1_str][0]) * dividing_coefficient
    delta_lon = (nav_data_dict[pos_time_shift_2_str][1] - nav_data_dict[pos_time_shift_1_str][1]) * dividing_coefficient
    return [nav_data_dict[pos_time_shift_1_str][0] + delta_lat, nav_data_dict[pos_time_shift_1_str][1] + delta_lon]         # [lat_float, lon_float]

def get_timestamp_str(timestamp_float):
    try:
        tmtime_tmp = datetime.datetime.fromtimestamp(timestamp_float)
        return tmtime_tmp.strftime('%Y-%m-%dT%H:%M:%S')
    except:
        return None

# file_name = 'DVR-2021-05-11_17-45-47' data_timeliness_UTC = '154548.000'
# Note: This function does not take into account changing the date (at midnight).
def get_simple_utc_start_time_list(file_name, data_timeliness_UTC):
    local_start_time_list = file_name.split('_')[1].split('.')[0].split('-')
    local_m = int(local_start_time_list[1])
    utc_m = int(data_timeliness_UTC[2:4])
    utc_h = int(data_timeliness_UTC[0:2])
    if utc_m < local_m:
        utc_h -= 1
    h = utc_h
    m = local_m
    s = int(local_start_time_list[2])
    return [h, m, s]

#=========================================================================================================================================
# main script:

print("SCRIPT BEGIN")
print('\n')

video_file_path_list = glob.glob(os.path.join(INPUT_DVR_FOLDER_PATH, '*' + VIDEO_FILES_EXTENSION))
nmea_file_path_list = glob.glob(os.path.join(INPUT_DVR_FOLDER_PATH, '*' + NMEA_FILES_EXTENSION))
video_file_names_list = [os.path.splitext(os.path.basename(video_file_path))[0] for video_file_path in video_file_path_list]
video_file_names_list.sort()
nmea_file_names_list = [os.path.splitext(os.path.basename(nmea_file_path))[0] for nmea_file_path in nmea_file_path_list]
nmea_file_names_list.sort()
if video_file_names_list != nmea_file_names_list:
    print ("The video_file_names_list and nmea_file_names_list are not identical.")
else:
    if not os.path.exists(OUTPUT_PIC_FOLDER_PATH):
        os.makedirs(OUTPUT_PIC_FOLDER_PATH)
    csv_file = open(os.path.join(OUTPUT_PIC_FOLDER_PATH, OUTPUT_CSV_FILE_NAME), 'w')
    csv_file.write(','.join(HEADER_TUPLE) + '\n')
    file_names_dict = {}
    for video_file_name in video_file_names_list:
        video_file_path = os.path.join(INPUT_DVR_FOLDER_PATH, video_file_name + VIDEO_FILES_EXTENSION)
        video_file_tmtime_float = os.path.getmtime(video_file_path)
        file_names_dict[video_file_name] = video_file_tmtime_float
    file_names_dict = dict(sorted(file_names_dict.items(), key=lambda item: item[1]))
    for file_name in file_names_dict.keys():
        print(str(file_name) + ' -> ' + str(file_names_dict[file_name]) + ' -> ' + str(get_timestamp_str(file_names_dict[file_name])))
        nmea_file_path = os.path.join(INPUT_DVR_FOLDER_PATH, file_name + NMEA_FILES_EXTENSION)
        f = open(nmea_file_path, 'r')
        nmea_text = f.read()
        f.close()
        nmea_rows_list = [t for t in nmea_text.split('\n') if t and (t.startswith('$GPGGA') or t.startswith('$GPRMC')) and check_mnea_data_line_CRC(t)]
        nav_data_dict = {}
        utc_start_time = None
        previous_h = None
        for nmea_row in nmea_rows_list:
            data_list = get_data_list(nmea_row)
            if data_list and len(data_list) == 4:
                if utc_start_time is None:
                    utc_start_time_list = get_simple_utc_start_time_list(file_name, data_list[0])
                    h = utc_start_time_list[0]
                    m = utc_start_time_list[1]
                    s = utc_start_time_list[2]
                    utc_start_time = h * 3600 + m * 60 + s
                    previous_h = h
                h = int(data_list[0][0:2])
                m = int(data_list[0][2:4])
                s = int(data_list[0][4:6])
                if h < previous_h:
                    h = previous_h + 1
                previous_h = h
                current_time = h * 3600 + m * 60 + s
                time_shift_str = str(current_time - utc_start_time)
                if time_shift_str not in nav_data_dict.keys():
                    nav_data_dict[time_shift_str] = data_list[1:]
                elif len(data_list[3]) > 0 and len(nav_data_dict[time_shift_str][2]) == 0:
                    nav_data_dict[time_shift_str] = data_list[1:]
        cap = cv2.VideoCapture(os.path.join(INPUT_DVR_FOLDER_PATH, file_name + VIDEO_FILES_EXTENSION))
        readed_frames_count = -1
        jpg_frames_count = 0
        frames_delay_sec = None
        previous_lat = None
        previous_lon = None
        distance = -1
        previous_work_percent = -1
        while cap.isOpened():
            ret, current_camera_frame = cap.read()
            readed_frames_count += 1
            work_percent = int(100 * readed_frames_count / cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if work_percent % 10 == 0 and work_percent > previous_work_percent:
                print(str(work_percent) + '%')
                previous_work_percent = work_percent
            if ret == True:
                if frames_delay_sec is None:
                    frames_delay_sec = float(1 / int(cap.get(cv2.CAP_PROP_FPS)))
                frame_pos = calculate_position_of_picture(nav_data_dict, readed_frames_count * frames_delay_sec)
                if frame_pos is None:
                    continue
                if previous_lat is not None and previous_lon is not None:
                    distance = calculate_distance_by_BL_WGS_84(previous_lat, previous_lon, frame_pos[0], frame_pos[1])
                if distance > MIN_DISTANCE_BETWEEN_PICTURES_m or distance < 0:
                    jpg_frames_count += 1
                    jpg_frames_count_str = str(jpg_frames_count)
                    zeros_count = 4 - len(jpg_frames_count_str)
                    if zeros_count > 0:
                        jpg_frames_count_str = zeros_count * '0' + jpg_frames_count_str
                    pic_file_name = file_name + '_' + jpg_frames_count_str + '.jpg'
                    cv2.imwrite(os.path.join(OUTPUT_PIC_FOLDER_PATH, pic_file_name), current_camera_frame)
                    csv_file.write(str(frame_pos[0]) + ',' + str(frame_pos[1]) + ',' + get_timestamp_str(file_names_dict[file_name] + readed_frames_count * frames_delay_sec) + ',' + pic_file_name + '\n')
                    previous_lat = frame_pos[0]
                    previous_lon = frame_pos[1]
            else:
                break
        print('readed frames: ' + str(readed_frames_count) + ' / ' + str(int(cap.get(cv2.CAP_PROP_FRAME_COUNT))))
        print('saved jpegs: ' + str(jpg_frames_count))
        cap.release()

    csv_file.close()
    
print('\n')
print('SCRIPT END')

#=========================================================================================================================================
