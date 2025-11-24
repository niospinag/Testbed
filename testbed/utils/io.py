

import numpy as np
import scipy.io as spio




def load_data_matlab(filename='', split_data=0,  shift_x=-200, scale_x=1.3, shift_y=-85, scale_y=30):
    '''
    Generate a function with the goal path.
    Args:
        filename: name of the .mat file to be loaded
        split_data: number of points between each iteration

    Returns:
        position: function to give a desired point in the loaded path
    '''
    assert isinstance(filename, str), "In the load_data_matlab function, the argument must be an string with the name of the file that containts the .mat file. Recieved type %r." % type(filename).__name__
    assert isinstance(split_data, int), "In the load_data_matlab function, the argument must be an integer with the number of the subdata needed between 2 points. Recieved type %r." % type(filename).__name__

    # mat = spio.loadmat('myData.mat', squeeze_me=True)
    mat = spio.loadmat(filename, squeeze_me=True)

    vhist = mat['vhist']  # structures need [()]
    vphist = mat['vphist']
    hist_pos = mat['hist_pos']
    zhist = mat['zhist']
    # zhist.astype=(float)
    # zphist = mat['zphist'] * scale_y + shift_y
    print('zhist', zhist.shape)
    print('hist_pos', hist_pos.shape)

    # T = mat['T']
    N = hist_pos.shape[0]
    horizon = hist_pos.shape[1]

    if split_data != 0:
        x_pos = np.zeros((N, (horizon-1)*split_data))
        y_pos = np.zeros((N, (horizon-1)*split_data))

        for j in np.arange(0, horizon-1, 1):
            print(j)

            dty = zhist[:, j+1].astype(float) - zhist[:, j].astype(float)
            dty = dty / (split_data)

            dtx = (hist_pos[:, j+1] - hist_pos[:, j]) / split_data
            # print('dty',dty)

            for k in range(split_data):
                x_pos[:, j*split_data + k] = hist_pos[:, j] + k*dtx
                y_pos[:, j*split_data + k] = zhist[:, j] + k*dty

                # print(f'x_pos {j*split_data + k}', x_pos[ag, j*split_data + k] )
                # print(f'y_pos {j*split_data + k}', y_pos[ag, j*split_data + k] )
        # print(x_pos.shape, y_pos.shape)
        # print(x_pos[:,200:])

    else:
        x_pos = hist_pos
        y_pos = zhist
        print(x_pos.shape, y_pos.shape)

    def position(i):
        # pos = [x_pos[:, i]*scale_x + shift_x,  y_pos[:, i]*scale_y+shift_y, np.zeros((6))]
        pos = np.array([x_pos[:, i]*scale_x + shift_x,
                       y_pos[:, i]*scale_y+shift_y, np.zeros((N))])
        print('pos', pos)
        return pos


    return position

