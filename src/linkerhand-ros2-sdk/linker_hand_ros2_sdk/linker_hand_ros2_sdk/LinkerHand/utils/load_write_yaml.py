#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
Author: HJX
Date: 2025-04-01 14:09:21
LastEditors: Please set LastEditors
LastEditTime: 2025-04-11 10:19:01
FilePath: /LinkerHand_Python_SDK/LinkerHand/utils/load_write_yaml.py
Description: 
symbol_custom_string_obkorol_copyright: 
'''
import os
import yaml


class LoadWriteYaml():
    def __init__(self):
        # Primary path: installed package directory (site-packages/.../LinkerHand/config)
        base_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_linkerhand_dir = os.path.abspath(os.path.join(base_dir, ".."))
        self.config_dir = os.path.join(pkg_linkerhand_dir, "config")

        # Fallback path: ament share directory (share/linker_hand_ros2_sdk/LinkerHand/config)
        if not os.path.exists(os.path.join(self.config_dir, "setting.yaml")):
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory("linker_hand_ros2_sdk")
                share_config_dir = os.path.join(share_dir, "LinkerHand", "config")
                if os.path.exists(os.path.join(share_config_dir, "setting.yaml")):
                    self.config_dir = share_config_dir
            except Exception:
                pass

        self.setting_path = os.path.join(self.config_dir, "setting.yaml")
        self.l7_positions = os.path.join(self.config_dir, "L7_positions.yaml")
        self.l10_positions = os.path.join(self.config_dir, "L10_positions.yaml")
        self.l20_positions = os.path.join(self.config_dir, "L20_positions.yaml")
        self.l21_positions = os.path.join(self.config_dir, "L21_positions.yaml")
        self.l25_positions = os.path.join(self.config_dir, "L25_positions.yaml")
        

    def load_setting_yaml(self):
        try:
            with open(self.setting_path, 'r', encoding='utf-8') as file:
                setting = yaml.safe_load(file)
                self.sdk_version = setting["VERSION"]
                self.left_hand_exists = setting['LINKER_HAND']['LEFT_HAND']['EXISTS']
                self.left_hand_names = setting['LINKER_HAND']['LEFT_HAND']['NAME']
                self.left_hand_joint = setting['LINKER_HAND']['LEFT_HAND']['JOINT']
                self.left_hand_force = setting['LINKER_HAND']['LEFT_HAND']['TOUCH']
                self.right_hand_exists = setting['LINKER_HAND']['RIGHT_HAND']['EXISTS']
                self.right_hand_names = setting['LINKER_HAND']['RIGHT_HAND']['NAME']
                self.right_hand_joint = setting['LINKER_HAND']['RIGHT_HAND']['JOINT']
                self.right_hand_force = setting['LINKER_HAND']['RIGHT_HAND']['TOUCH']
                self.password = setting['PASSWORD']
        except Exception as e:
            setting = None
            print(f"Error reading setting.yaml: {e}")
        self.setting = setting
        return self.setting
    
    def load_action_yaml(self,hand_joint="",hand_type=""):
        if hand_joint == "L20":
            action_path = self.l20_positions
        elif hand_joint == "L10":
            action_path = self.l10_positions
        elif hand_joint == "L25":
            action_path = self.l25_positions
        elif hand_joint == "L21":
            action_path = self.l21_positions
        elif hand_joint == "L7":
            action_path = self.l7_positions
            print(action_path)
        try:
            with open(action_path, 'r', encoding='utf-8') as file:
                yaml_data = yaml.safe_load(file)
                if hand_type == "left":
                    self.action_yaml = yaml_data["LEFT_HAND"]
                else:
                    self.action_yaml = yaml_data["RIGHT_HAND"]
        except Exception as e:
            self.action_yaml = None
            print(f"yaml配置文件不存在: {e}")
        return self.action_yaml 

    def write_to_yaml(self, action_name, action_pos,hand_joint="",hand_type=""):
        a = False
        if hand_joint == "L20":
            action_path = self.l20_positions
        elif hand_joint == "L10":
            action_path = self.l10_positions
        elif hand_joint == "L7":
            action_path = self.l7_positions
        elif hand_joint == "L21":
            action_path = self.l21_positions
        elif hand_joint == "L25":
            action_path = self.l25_positions
        try:
            with open(action_path, 'r', encoding='utf-8') as file:
                yaml_data = yaml.safe_load(file)
                print(yaml_data)
            if hand_type == "left":
                if yaml_data["LEFT_HAND"] == None:
                    yaml_data["LEFT_HAND"] = []
                yaml_data["LEFT_HAND"].append({"ACTION_NAME": action_name, "POSITION": action_pos})
            elif hand_type == "right":
                if yaml_data["RIGHT_HAND"] == None:
                    yaml_data["RIGHT_HAND"] = []
                yaml_data["RIGHT_HAND"].append({"ACTION_NAME": action_name, "POSITION": action_pos})
            with open(action_path, 'w', encoding='utf-8') as file:
                yaml.safe_dump(yaml_data, file, allow_unicode=True)
            a = True
        except Exception as e:
            a = False
            print(f"Error writing to yaml file: {e}")
        return a
        
