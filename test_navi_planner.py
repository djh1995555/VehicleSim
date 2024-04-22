

import os
import yaml
import plotly.graph_objs as go
from planner.naive_planner import NaivePlanner

def plot_trajectory(trajectory):
        reference_x = []
        reference_y = []
        curvatures = []

        for trajectory_segment in trajectory.trajectory_segments:
            for trajectory_point in trajectory_segment.trajectory_points:
                reference_x.append(trajectory_point.x)
                reference_y.append(trajectory_point.y)
                curvatures.append(trajectory_point.curvature)


        scatter1 = go.Scatter(
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
        fig.add_trace(scatter1)

        fig.update_layout(
            title='Trajectories Tracking',
            xaxis_title='X Axis',
            yaxis_title='Y Axis'
        )
        fig.write_html(os.path.join('trajectory_comparison.html'))


        curvature_scatter = go.Scatter(
            x = [x for x in range(len(curvatures))],
            y = curvatures,
            name = 'curvature',
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
        curvature_fig = go.Figure()
        curvature_fig.add_trace(curvature_scatter)

        curvature_fig.update_layout(
            title='curvature',
        )
        curvature_fig.write_html(os.path.join('curvature.html'))

dir_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__))))
config_file_name = 'simulator_config.yaml'
simulator_config_filepath = os.path.join(dir_path,'config', config_file_name)
with open(simulator_config_filepath, 'r') as f:
    simulator_config = yaml.load(f,Loader=yaml.Loader)

navie_planner = NaivePlanner(simulator_config)
trajectory = navie_planner.generate_trajectory()
plot_trajectory(trajectory)

