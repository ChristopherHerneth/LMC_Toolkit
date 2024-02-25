import plotly.graph_objs as go
import numpy as np

def addArrow(fig, start, vec, text=None, color='red', arrow_tip_ratio=0.3, arrow_starting_ratio=0.98):
    vec = vec + start # vec needs to be the vectors starting from start not from the origin

    fig.add_trace(go.Scatter3d(
        x = [start[0]],
        y = [start[1]],
        z = [start[2]],
        text = text,
        mode='markers+text',
        marker=dict(
            color=color, 
            size=5,
        )
    ))

    x_lines = list()
    y_lines = list()
    z_lines = list()

    for i in [start, vec]:
        x_lines.append(i[0])
        y_lines.append(i[1])
        z_lines.append(i[2])
    x_lines.append(None)
    y_lines.append(None)
    z_lines.append(None)

    ## set the mode to lines to plot only the lines and not the balls/markers
    fig.add_trace(go.Scatter3d(
        x=x_lines,
        y=y_lines,
        z=z_lines,
        mode='lines',
        line = dict(width = 2, color = color)
    ))


    ## the cone will point in the direction of vector field u, v, w 
    ## so we take this to be the difference between each pair 

    ## then hack the colorscale to force it to display the same color
    ## by setting the starting and ending colors to be the same

    fig.add_trace(go.Cone(
        x=[start[0] + arrow_starting_ratio*(vec[0] - start[0])],
        y=[start[1] + arrow_starting_ratio*(vec[1] - start[1])],
        z=[start[2] + arrow_starting_ratio*(vec[2] - start[2])],
        u=[arrow_tip_ratio*(vec[0] - start[0])],
        v=[arrow_tip_ratio*(vec[1] - start[1])],
        w=[arrow_tip_ratio*(vec[2] - start[2])],
        showlegend=False,
        showscale=False,
        colorscale=[[0, color], [1, color]]
        ))

    return fig

def addMarker(fig, pos, color='blue', name=''):
    fig.add_trace(go.Scatter3d(
            x=[pos[0]],
            y=[pos[1]],
            z=[pos[2]],
            mode='markers+text',
            text=name,
            marker=dict(
                size=5,
                color=color,
            )
    ))

    return fig

def plot_hand(fig, pos_markers, color='blue', names=np.arange(0, 26)):
    if fig is None:
        fig = go.Figure()
    fig.add_trace(go.Scatter3d(
            x=pos_markers[:, 0],
            y=pos_markers[:, 1],
            z=pos_markers[:, 2],
            mode='markers+text',
            text=names,
            marker=dict(
                size=5,
                color=color,
            )
    ))

    fig.update_layout(
        width=1024,
        height=1024,
        scene = dict(
            xaxis=dict(),
            yaxis=dict(),
            zaxis=dict(),
            aspectmode='data', #this string can be 'data', 'cube', 'auto', 'manual'
            #a custom aspectratio is defined as follows:
            aspectratio=dict(x=1, y=1, z=1)
        ), 
        scene_camera = dict(
            up=dict(x=0, y=-0, z=20),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=-2, y=-0.5, z=3)
            )
    )

    return fig

def plot_hand_and_marker(pos_markers, marker, fig=None, color='red', name=''):
    if fig == None:
        fig = plot_hand(pos_markers)
    fig.add_trace(go.Scatter3d(
            x=[marker[0]],
            y=[marker[1]],
            z=[marker[2]],
            mode='markers+text',
            text=name,
            marker=dict(
                size=5,
                color=color,
            )
    ))

    return fig