# -*- coding: utf-8 -*-
# https://github.com/ros2/rosbag2/issues/473

import sqlite3
import csv
import sys
import yaml
import os
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":
        bag_file = sys.argv[1]+'/'+sys.argv[1]+'_0.db3'
        parser = BagFileParser(bag_file)
        aux = bag_file.split('/')
        aux.pop()
        metadata_file = ''.join(aux)+'/metadata.yaml'
        topic_list = []
        with open(metadata_file, 'r') as file:
            topics_file = yaml.load(file, Loader=yaml.FullLoader)
            topics_info = topics_file.get("rosbag2_bagfile_information").get("topics_with_message_count")
            for i in range(len(topics_info)):
                topic_aux = topics_info[i]['topic_metadata'].get("name")
                topic_list.append(topic_aux)
        print(topic_list)

        # Range0
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_range0.csv'
        topic_name = '/'+sys.argv[2]+'/range0'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'radiation_type', 'field_of_view', 'min_range', 'max_range', 'range']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0], data[i][1].radiation_type, data[i][1].field_of_view, data[i][1].min_range, data[i][1].max_range, data[i][1].range]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_range0.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Range1
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_range1.csv'
        topic_name = '/'+sys.argv[2]+'/range1'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'radiation_type', 'field_of_view', 'min_range', 'max_range', 'range']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0], data[i][1].radiation_type, data[i][1].field_of_view, data[i][1].min_range, data[i][1].max_range, data[i][1].range]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_range1.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Range2
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_range2.csv'
        topic_name = '/'+sys.argv[2]+'/range2'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'radiation_type', 'field_of_view', 'min_range', 'max_range', 'range']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0], data[i][1].radiation_type, data[i][1].field_of_view, data[i][1].min_range, data[i][1].max_range, data[i][1].range]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_range2.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Range3
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_range3.csv'
        topic_name = '/'+sys.argv[2]+'/range3'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'radiation_type', 'field_of_view', 'min_range', 'max_range', 'range']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0], data[i][1].radiation_type, data[i][1].field_of_view, data[i][1].min_range, data[i][1].max_range, data[i][1].range]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_range3.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Range4
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_range4.csv'
        topic_name = '/'+sys.argv[2]+'/range4'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'radiation_type', 'field_of_view', 'min_range', 'max_range', 'range']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0], data[i][1].radiation_type, data[i][1].field_of_view, data[i][1].min_range, data[i][1].max_range, data[i][1].range]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_range4.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)
        
        # cmd_vel
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_cmd_vel.csv'
        topic_name = '/'+sys.argv[2]+'/cmd_vel'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].linear.x, data[i][1].linear.y, data[i][1].linear.z,
                       data[i][1].angular.x, data[i][1].angular.y, data[i][1].angular.z]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_cmd_vel.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Odom
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_odom.csv'
        topic_name = '/'+sys.argv[2]+'/odom'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].pose.pose.position.x, data[i][1].pose.pose.position.y, data[i][1].pose.pose.position.z,
                       data[i][1].pose.pose.orientation.x, data[i][1].pose.pose.orientation.y, data[i][1].pose.pose.orientation.z, data[i][1].pose.pose.orientation.w,
                       data[i][1].twist.twist.linear.x, data[i][1].twist.twist.linear.y, data[i][1].twist.twist.linear.z,
                       data[i][1].twist.twist.angular.x, data[i][1].twist.twist.angular.y, data[i][1].twist.twist.angular.z]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_odom.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Ground Truth
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_groundtruth.csv'
        topic_name = '/'+sys.argv[2]+'/ground_truth'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].pose.pose.position.x, data[i][1].pose.pose.position.y, data[i][1].pose.pose.position.z,
                       data[i][1].pose.pose.orientation.x, data[i][1].pose.pose.orientation.y, data[i][1].pose.pose.orientation.z, data[i][1].pose.pose.orientation.w,
                       data[i][1].twist.twist.linear.x, data[i][1].twist.twist.linear.y, data[i][1].twist.twist.linear.z,
                       data[i][1].twist.twist.angular.x, data[i][1].twist.twist.angular.y, data[i][1].twist.twist.angular.z]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_groundtruth.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_pose.csv'
        topic_name = '/'+sys.argv[2]+'/pose'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_pose.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Camera Info
        # dir_file = sys.argv[1]+'/'+sys.argv[2]+'_cam_info.csv'
        # topic_name = '/'+sys.argv[2]+'/camera/camera_info'
        # if (topic_name in topic_list and not os.path.isfile(dir_file)):
        #     print('Converting topic: '+topic_name+'...')
        #     data = parser.get_messages(topic_name)[:][:]
        #     header = ['Timestamp', 'height', 'width', 'distortion_model', 'D', 'K', 'R', 'P', 'binning_x', 'binning_y']
        #     data_csv = []
        #     for i in range(len(parser.get_messages(topic_name))):
        #         aux = [data[i][0],
        #                data[i][1].height, data[i][1].width, data[i][1].distortion_model,
        #                data[i][1].D, data[i][1].K, data[i][1].R, data[i][1].P,
        #                data[i][1].binning_x, data[i][1].binning_y]
        #         data_csv.append(aux)

        #     aux = bag_file.split('/')
        #     aux.pop()
        #     dir = ''.join(aux)+'/'+sys.argv[2]+'_cam_info.csv'
        #     with open(dir, 'w', encoding='UTF8', newline='') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(header)
        #         writer.writerows(data_csv)

        # IMU
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_imu.csv'
        topic_name = '/'+sys.argv[2]+'/imu/data'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w,
                       data[i][1].angular_velocity.x, data[i][1].angular_velocity.y, data[i][1].angular_velocity.z,
                       data[i][1].linear_acceleration.x, data[i][1].linear_acceleration.y, data[i][1].linear_acceleration.z]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_imu.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Goal Pose
        dir_file = sys.argv[1]+'/'+sys.argv[2]+'_goalpose.csv'
        topic_name = '/'+sys.argv[2]+'/goal_pose'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                aux = [data[i][0],
                       data[i][1].position.x, data[i][1].position.y, data[i][1].position.z,
                       data[i][1].orientation.x, data[i][1].orientation.y, data[i][1].orientation.z, data[i][1].orientation.w]
                data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/'+sys.argv[2]+'_goalpose.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)

        # Performance_metrics
        dir_file = sys.argv[1]+'/performance_metrics.csv'
        topic_name = '/performance_metrics'
        if (topic_name in topic_list and not os.path.isfile(dir_file)):
            print('Converting topic: '+topic_name+'...')
            data = parser.get_messages(topic_name)[:][:]
            header = ['Timestamp', 'real_time_factor', 'name', 'sim_update_rate', 'real_update_rate', 'fps']
            data_csv = []
            for i in range(len(parser.get_messages(topic_name))):
                for j in range(len(data[i][1].sensors)):
                    aux = [data[i][0],
                        data[i][1].real_time_factor, data[i][1].sensors[j].name, data[i][1].sensors[j].sim_update_rate,
                        data[i][1].sensors[j].real_update_rate, data[i][1].sensors[j].fps]
                    data_csv.append(aux)

            aux = bag_file.split('/')
            aux.pop()
            dir = ''.join(aux)+'/performance_metrics.csv'
            with open(dir, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_csv)