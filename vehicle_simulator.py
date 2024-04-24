#!/usr/bin/env python
import argparse
import copy
import datetime
import math
import os
import shutil
import numpy as np
import pandas as pd
from collections import OrderedDict 
import yaml
import plotly.graph_objs as go
import plotly.offline as offline
from utils.report_plotter import ReportPlotter
from planner.naive_planner import NaivePlanner
from dynamic_model.vehicle_dynamic_model import *
from controller.vehicle_controller import *
from proto.controller_pb2 import ControllerOutput
from proto.trajectory_pb2 import Trajectory
from proto.vehicle_state_pb2 import VehicleState

class VehicleSimulator():
    def __init__(self,args):
        simulator_config_filepath = os.path.join(dir_path,'config', args.config)
        with open(simulator_config_filepath, 'r') as f:
            simulator_config = yaml.load(f,Loader=yaml.Loader)

        self._args = args
        self._simulation_time = simulator_config['simulation_time']
        self._sample_time = simulator_config['sample_time']
        self._ref_length = simulator_config['ref_length']

        
        self._navie_planner = NaivePlanner(simulator_config)

        self._vehicle_dynamic_model = VehicleDynamicModel(simulator_config)
        self._vehicle_dynamic_model.reset(simulator_config['init_v'], simulator_config['init_pitch'])

        self._vehicle_controller = VehicleController(simulator_config)

        self._report_plotter = ReportPlotter('ReportGenerator')
        self._debug_info = OrderedDict()

        self._trajectory = self._navie_planner.generate_trajectory()
        self._controller_output_history = []
        self._vehicle_state_history = []
        self._segment_history = []
        self._data_selected = OrderedDict()

    def run(self):
        is_finish = False
        count = 0
        while(not is_finish and count < int(self._simulation_time/self._sample_time)):
            controller_output = ControllerOutput()
            state = self._vehicle_dynamic_model.get_state()
            self._vehicle_state_history.append(copy.deepcopy(state))
            is_finish, trajectory_segment = self._navie_planner.get_segment(state)
            self._segment_history.append(trajectory_segment.segment_id)
            self._vehicle_controller.compute_control_cmd(state, trajectory_segment, controller_output)
            self._controller_output_history.append(copy.deepcopy(controller_output))
            self._vehicle_dynamic_model.update(controller_output.control_cmd)
            count += 1

        if(count >=int(self._simulation_time/self._sample_time)):
            print("running out of time!")
        if(is_finish):
            print("simulation is finished!")

    def plot_trajectory(self):
        is_3d = False
        vehicle_x = []
        vehicle_y = []
        vehicle_v = []
        for i in range(len(self._vehicle_state_history)):
            state = self._vehicle_state_history[i]
            vehicle_x.append(state.x)
            vehicle_y.append(state.y)
            vehicle_v.append(state.v) 
        
        if(is_3d):
            tracking_result_scatter = go.Scatter3d(
                x = vehicle_x,
                y = vehicle_y,
                z = vehicle_v,
                name = 'Vehicle Trajectory',
                mode = 'lines',
                marker = dict(
                    size = 10,
                    color = 'rgba(255, 0, 0, 0.8)'  # 红色轨迹
                ),
                line = dict(
                    width = 2,
                    color = 'rgba(255, 0, 0, 1)'  # 轨迹线的颜色
                )
            )
        else:
            tracking_result_scatter = go.Scatter(
                x = vehicle_x,
                y = vehicle_y,
                name = 'Vehicle Trajectory',
                mode = 'lines',
                marker = dict(
                    size = 10,
                    color = 'rgba(255, 0, 0, 0.8)'  # 红色轨迹
                ),
                line = dict(
                    width = 2,
                    color = 'rgba(255, 0, 0, 1)'  # 轨迹线的颜色
                )
            )

        reference_x = []
        reference_y = []
        reference_v = []

        for trajectory_segment in self._trajectory.trajectory_segments:
            for trajectory_point in trajectory_segment.trajectory_points:
                reference_x.append(trajectory_point.x)
                reference_y.append(trajectory_point.y)
                reference_v.append(trajectory_point.v)

        if(is_3d):
            reference_scatter = go.Scatter3d(
                x = reference_x,
                y = reference_y,
                z = reference_v,
                name = 'Reference',
                mode = 'lines',  # 使用线和标记表示轨迹
                marker = dict(
                    size = 10,
                    color = 'rgba(0, 128, 255, 0.8)'  # 蓝色轨迹
                ),
                line = dict(
                    width = 2,
                    color = 'rgba(0, 128, 255, 1)'  # 轨迹线的颜色
                )
            )
        else:
            reference_scatter = go.Scatter(
                x = reference_x,
                y = reference_y,
                name = 'Reference',
                mode = 'lines',  # 使用线和标记表示轨迹
                marker = dict(
                    size = 10,
                    color = 'rgba(0, 128, 255, 0.8)'  # 蓝色轨迹
                ),
                line = dict(
                    width = 2,
                    color = 'rgba(0, 128, 255, 1)'  # 轨迹线的颜色
                )
            )
        

        

        fig = go.Figure()
        fig.add_trace(reference_scatter)
        fig.add_trace(tracking_result_scatter)
        fig.update_layout(
            scene=dict(
                xaxis=dict(showgrid=True),
                yaxis=dict(showgrid=True),
                zaxis=dict(showgrid=True)
            )
        )

        fig.update_layout(
            title='Trajectories Tracking',
            xaxis_title='X Axis',
            yaxis_title='Y Axis'
        )
        fig.write_html(os.path.join(args.output_dir,'trajectory_comparison.html'))

    def select_target_signal(self):
        with open(self._args.target_signal_filepath, 'r') as f:
            target_signals = yaml.load(f, Loader=yaml.Loader)
        
        target_panel = dict(target_signals["target_panels"])
        for sub_panel_name, sub_panel in target_panel.items():
            sub_dict = OrderedDict()
            for signal_full_name in sub_panel:
                channel, signal = signal_full_name.split(":")
                if(channel == "vehicle_state"):
                    data_source = self._vehicle_state_history
                    target_data = [getattr(data, signal) for data in data_source]
                elif(channel == "control_cmd"):
                    data_source = self._controller_output_history
                    target_data = [getattr(data.control_cmd, signal) for data in data_source]
                elif(channel == "control_debug"):
                    data_source = self._controller_output_history
                    target_data = [getattr(data.control_debug, signal) for data in data_source]      
                elif(channel == "trajectory"):
                    data_source = self._segment_history    
                    target_data = [data for data in data_source]   
                            

                time = [self._sample_time* i for i in range(len(target_data))]
                sub_dict[signal] = (target_data, time)              
            self._data_selected[sub_panel_name] = sub_dict

    def plot_signal(self, name = 'output'):
        output_dir = args.output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        self.select_target_signal()
        self.plot_trajectory()

        src_config = os.path.join(dir_path, 'config', args.config)
        tgt_config = os.path.join(output_dir,args.config)

        shutil.copyfile(src_config, tgt_config) 

        subplot_figure = None
        plot_html_str = ""
        figure_list = []
        for sub_panel_name, signals in self._data_selected.items():
            # print('segment_name:{}'.format(segment_name))
            legend_list = []
            data_list = []
            time_list = []
            for signal_name, (data, time) in signals.items():
                # print(' segment_name:{}, len:{}'.format(segment_name, len(data)))
                legend_list.append(signal_name)
                data_list.append(np.array(data))
                time_list.append(np.array(time))
            subplot = self._report_plotter.plot_figure_plotly(x_list = time_list, 
                                                        y_list = data_list,
                                                        legend_list = legend_list,
                                                        x_label = 'timestamp',
                                                        y_label = 'value',
                                                        title = '{}'.format(sub_panel_name),
                                                        legend_prefix = '',
                                                        figure_height= 300,)
            figure_list.append(subplot)

        subplot_figure_list = [(i + 1, 1, fig) for i, fig in enumerate(figure_list)]
        subplot_figure = self._report_plotter.append_figure_to_subplot_plotly(subplot_figure_list, 
                                                                            row_num = len(figure_list), 
                                                                            col_num = 1, 
                                                                            template="plotly_dark", 
                                                                            subplot_fig=subplot_figure,
                                                                            figure_height = 300,
                                                                            vertical_spacing = 0.02)
        plot_html_str += self._report_plotter.get_fuel_fig_html_str({'output_name': subplot_figure})
        html_str = self._report_plotter.generate_html_fuel_report(plot_html_str)
        with open(os.path.join(output_dir,'{}.html'.format(name)), 'w') as f:
            f.write(html_str)

def main(args):
    vehicle_simulator = VehicleSimulator(args)
    vehicle_simulator.run()
    vehicle_simulator.plot_signal()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Sim Vehicle Dynamic Model')
    dir_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__))))
    print('file path:{}'.format(dir_path))
    parser.add_argument('--config', default='simulator_config.yaml', type=str)
    parser.add_argument('--output-dir', default=os.path.join(dir_path,'output','{}'.format(datetime.datetime.now())))
    parser.add_argument('--target-signal-filepath', default=os.path.join(dir_path,'config/plot_signal.yaml'), type=str)
    args = parser.parse_args()
  
    main(args)