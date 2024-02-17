import plotly.graph_objs as go

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


    '''
        takes the path to a .sto file and displayes the hardcoded angles and moments
    '''
    sto_data = readFile_ID_sto(path, filename)
    data_dict_ID = sto_data[-1]
    timeseries = data_dict_ID['time']

    print(data_dict_ID.keys())

    fig = go.Figure()

    hand = 'R'
    #relevant = ['elbow_flexion_' +hand.lower()+ '_moment', 'pro_sup_' +hand.lower()+ '_moment', 'wrist_hand_r3_' +hand.lower()+ '_moment', 'wrist_hand_r1_' +hand.lower()+ '_moment']
    #labels = ['elbow_flex+_ext-_moment_'+hand.lower(), 'pro+_sup-_moment_'+hand.lower(), 'wrist_flex+_ext-_moment_'+hand.lower(), 'wrist_dev_ulnar+_rad-_moment_'+hand.lower()] # the order is important!!

    relevant = ['elv_angle_'+hand.lower()+'_moment', 'shoulder_elv_'+hand.lower()+ '_moment', 'shoulder_rot_'+hand.lower()+ '_moment', 
                'elbow_flexion_'+hand.lower()+ '_moment', 'pro_sup_'+hand.lower()+ '_moment', 
                'wrist_hand_r1_'+hand.lower()+ '_moment', 'wrist_hand_r3_'+hand.lower()+ '_moment']
    labels = ['shoulder_flex+_ext-_moment', 'shoulder_abd+_add-_moment', 'shoulder_rotint+_ext-_moment', 
              'elbow_flex+_ext-_moment', 'pro+_sup-_moment', 
              'wrist_flex+_ext-_moment', 'wrist_dev_ulnar+_rad-_moment'] # the order is important!!
    
    #'shoulder1_r2_'+hand.lower()+ '_moment', 

    for r, l in zip(relevant, labels):
        fig.add_trace(go.Scatter(
                        x=timeseries,
                        y=data_dict_ID[r],
                        name=l,
                        mode="lines",
                        )
        )

    fig.update_layout(
        width=800,
        height=500,
        title='Subject Forces',
        xaxis_title='time [s]',
        yaxis_title='joint torques [Nm]',
        #legend_title="Legend Title",
    )

    fig.show()