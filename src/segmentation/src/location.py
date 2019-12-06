


'''
This determines the ball's location and velocity with respect to the camera.
Default depth units = 1 mm

'''
fps = 30
z0 = 103.1875 #[mm]
r0 = 153.6826171875

r1 = (60.24595642089844 + 61.16786193847656)/2
z1 = z0 + 203.2 #[mm]

r2 = 38.5520133972168
z2 = z0 + 381 #[mm]

#tuples of the radius of the ball and the corresponding height
def exponential(radius):
    a = 4.42168598
    b = 0.02137836
    c = -1.26518199
    return

def parabolic(radius):
    a0 = 2.59936585e+01
    a1 = -3.72791626e-01
    a2 = 1.32533945e-03
    return a2*radius*radius + a1*radius + a0

def get_height(radius):
    return parabolic(radius)

def get_velocity(x_old, y_old, old_radius, x_new, y_new, new_radius, fps=30):
    '''
    Gets the velocity [m/s] based on the radius of the ball in the current and the previous frame.
    Input: the previous frame's x-y position and radius, the new frame's x-y position and radius, the frames per second of the camera. All integers
    Output: The ball's velocity vector, 3x1 [v_x, v_y, v_z]
    '''

    v_x = x_old - x_new
    v_y = y_old - y_new
    v_z = old_radius - new_radius



    return
