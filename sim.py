from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import argparse
import sys

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation
import numpy as np

parser = argparse.ArgumentParser(description='Simulate a multi-link pendulum')
parser.add_argument('--nlinks', metavar='N', type=int, default=1, help='number of links')
parser.add_argument('--timestep', metavar='dT', type=float, default=0.01, help='timestep in seconds')
parser.add_argument('--scv', metavar='Kp', type=float, default=2, help='constant value for velocity stabilization')
parser.add_argument('--scp', metavar='Kd', type=float, default=0.2, help='constant value for position stabilization')
parser.add_argument('--gsv', metavar='KGp', type=float, default=100, help='ground stiffness factor for velocity')
parser.add_argument('--gsp', metavar='KGd', type=float, default=100, help='ground stiffness factor for position')

args = parser.parse_args()

window = 0     # number of the glut window
theta = 0.0
simTime = 0
dT = args.timestep
G = 9.8
simRun = True
RAD_TO_DEG = 180.0/3.1416
nLinks = args.nlinks
Kp = args.scv
Kd = args.scp
Kf = 0.05 * nLinks + 0.01
KGp = args.gsv
KGd = args.gsp
groundPlane = 0.05  # I couldn't make the ground opaque, so I'm setting the ground to be a little bit higher

#####################################################
#### Link class, i.e., for a rigid body
#####################################################
times = 0
class Link:
        color=[0,0,0]    ## draw color
        size=[1,1,1]     ## dimensions
        mass = 1.0       ## mass in kg
        izz = 1.0        ## moment of inertia about z-axis
        theta=0          ## 2D orientation  (will need to change for 3D)
        omega=0          ## 2D angular velocity
        posn=np.array([0.0,0.0,0.0])     ## 3D position (keep z=0 for 2D)
        vel=np.array([0.0,0.0,0.0])      ## initial velocity
        def draw(self):      ### steps to draw a link
                glPushMatrix()                                            ## save copy of coord frame
                glTranslatef(self.posn[0], self.posn[1], self.posn[2])    ## move 
                glRotatef(self.theta[0]*RAD_TO_DEG,  1, 0, 0)                             ## rotate
                glRotatef(self.theta[1]*RAD_TO_DEG,  0, 1, 0)                             ## rotate
                glRotatef(self.theta[2]*RAD_TO_DEG,  0, 0, 1)                             ## rotate
                glScale(self.size[0], self.size[1], self.size[2])         ## set size
                glColor3f(self.color[0], self.color[1], self.color[2])    ## set colour
                DrawCube()                                                ## draw a scaled cube
                glPopMatrix()                                             ## restore old coord frame

#####################################################
#### main():   launches app
#####################################################

def main():
        global window
        global links
        global nLinks   
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)     # display mode 
        glutInitWindowSize(640, 480)                                  # window size
        glutInitWindowPosition(0, 0)                                  # window coords for mouse start at top-left
        window = glutCreateWindow("CPSC 526 Simulation Template")
        glutDisplayFunc(DrawWorld)       # register the function to draw the world
        # glutFullScreen()               # full screen
        glutIdleFunc(SimWorld)          # when doing nothing, redraw the scene
        glutReshapeFunc(ReSizeGLScene)   # register the function to call when window is resized
        glutKeyboardFunc(keyPressed)     # register the function to call when keyboard is pressed
        InitGL(640, 480)                 # initialize window
        
        links = [Link() for _ in range(nLinks)]
        resetSim()
        
        glutMainLoop()                   # start event processing loop

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

initialTheta = [0, 0, np.pi/4]
# initialPosn = np.array([0.0, 0.0, 0.0])
linkHeight = 1.0
fixedPoint = np.array([0.0, linkHeight/2, 0])

def resetSim():
        global links
        global simTime, simRun

        printf("Simulation reset\n")
        simRun = True
        simTime = 0
        lastPosition = (1 * nLinks - .2) * np.array([0.0, linkHeight, 0.0])
        # link1.size=[0.04, 1.0, 0.12]
        for i, link in enumerate(links):
                link.size = np.array([0.1, linkHeight, 0.1])
                link.color = [1, 0.9, 0.9]
                # if i == nLinks-1:
                #         link.theta = np.array(initialTheta)
                # else:
                #         link.theta = np.array([0.0, 0.0, 0.0])
                # link.posn = initialPosn - (i-nLinks+.2) * np.array([0.0, linkHeight, 0.0]) - np.matmul(getRotMatrix(link.theta), fixedPoint)
                link.theta = np.array(initialTheta)
                link.posn = lastPosition - np.matmul(getRotMatrix(link.theta), fixedPoint)
                link.initialTheta = link.theta
                link.initialPosn = link.posn
                lastPosition = link.posn - np.matmul(getRotMatrix(link.theta), fixedPoint)
                link.vel = np.array([0.0, 0.0, 0.0])
                link.omega = np.array([0.0, 0.0, 0.0])        ## radians per second

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def keyPressed(key,x,y):
    global simRun
    ch = key.decode("utf-8")
    if ch == ' ':                #### toggle the simulation
            if (simRun == True):
                 simRun = False
            else:
                 simRun = True
    elif ch == chr(27):          #### ESC key
            sys.exit()
    elif ch == 'q':              #### quit
            sys.exit()
    elif ch == 'r':              #### reset simulation
            resetSim()

#####################################################
#### SimWorld():  simulates a time step
#####################################################

def getRotMatrix(theta):
        return  np.matmul(
                np.array([
                        [np.cos(theta[2]), -np.sin(theta[2]), 0],
                        [np.sin(theta[2]), np.cos(theta[2]), 0],
                        [0, 0, 1],
                ]),
                np.matmul(
                        np.array([
                                [np.cos(theta[1]), 0, np.sin(theta[1])],
                                [0, 1, 0],
                                [-np.sin(theta[1]), 0, np.cos(theta[1])],
                        ]),
                        np.array([
                                [1, 0, 0],
                                [0, np.cos(theta[0]), -np.sin(theta[0])],
                                [0, np.sin(theta[0]), np.cos(theta[0])],
                        ])
                )
        )


def matrixStack(matrices):
        return np.vstack([np.hstack(r) for r in matrices])


def cProdMat(x):
        return np.array([
                [0, -x[2], x[1]],
                [x[2], 0, -x[0]],
                [-x[1], x[0], 0],
        ])


def generalizedDiag(matrices, offset=0):
        n_rows = sum([m.shape[1] for m in matrices]) + offset * (len(matrices)-1)
        rows = []
        past_rows = 0
        for m in matrices:
                rows.append(np.hstack([
                        np.zeros((m.shape[0], past_rows)),
                        m,
                        np.zeros((m.shape[0], n_rows-past_rows-m.shape[1])),
                ]))
                past_rows += m.shape[1] + offset
        return np.vstack(rows)


def getPosAndVelInWorld(link, p_local):
        p_world = link.posn + np.matmul(getRotMatrix(link.theta), p_local)
        vel = link.vel + np.matmul(link.omega, p_world)
        return p_world, vel


def getLowestPointWithVel(link, points):
        points_w = [getPosAndVelInWorld(link, p) for p in points]
        return sorted(points_w, key=lambda x: x[0][1])[0]


def SimWorld():
        global simTime, dT, simRun, fixedPoint
        global links, Kp, Kd, Kf, G, KGd, KGp, groundPlane

        deltaTheta = 2.4
        if (simRun==False):             ## is simulation stopped?
                return

            #### solve for the equations of motion (simple in this case!)

            ####  for the constrained one-link pendulum, and the 4-link pendulum,
            ####  you will want to build the equations of motion as a linear system, and then solve that.
            ####  Here is a simple example of using numpy to solve a linear system.
        # a = np.array([[2, -4, 4], [34, 3, -1], [1, 1, 1]])
        # omega_vec = [0, 0, link1.omega]
        r_local = fixedPoint
        gen_mass = []
        J = []
        # J2 = []
        b_mass = []
        b_cons = []
        last_weird_term = 0
        last_constrained_point_velocity = np.array([0.0, 0.0, 0.0])
        last_constrained_point_posn = links[0].initialPosn + np.matmul(getRotMatrix(links[0].theta), r_local)
        for i, link in enumerate(links):
                I_local = np.diag(
                        link.mass / 12 * (sum(np.array(link.size) ** 2) - np.array(link.size) ** 2)
                )
                R = getRotMatrix(link.theta)
                r_world = np.matmul(R, r_local)
                I_world = np.matmul(R, np.matmul(I_local, np.linalg.inv(R)))
                # print(r_world)
                # z = np.zeros((3,3))
                gen_mass.append(
                        generalizedDiag([
                                np.diag([link.mass] * 3),
                                I_world
                        ])
                )
                if i != len(links)-1:
                        J.append(
                                matrixStack([
                                        [np.diag([-1] * 3), np.diag([1] * 3)],
                                        [-cProdMat(r_world), -cProdMat(r_world)],
                                ])
                        )
                else:
                        J.append(
                                matrixStack([
                                        [np.diag([-1] * 3)],
                                        [-cProdMat(r_world)],
                                ])
                        )
                diff_velocity = last_constrained_point_velocity - (link.vel + np.cross(link.omega, r_world))
                diff_position = last_constrained_point_posn - (link.posn + r_world)
                last_constrained_point_velocity = link.vel - np.cross(link.omega, r_world)
                last_constrained_point_posn = (link.posn - r_world)
                # diff_velocity = 0
                # diff_position = 0
                weird_term = np.cross(link.omega, np.cross(link.omega, r_world))
                low_p, low_v = getLowestPointWithVel(link, [
                        link.size * [+0.5, -0.5, 0],
                        link.size * [-0.5, -0.5, 0],
                ])
                diff = low_p[1] - groundPlane
                ground_force_value = (i == nLinks-1) * (low_v[1] <= 0) * (diff <= 0) * (-1 * KGp * diff - KGd * low_v[1]) * np.array([0, 1.0, 0])
                ground_friction_value = (i == nLinks-1) * (low_v[1] <= 0) * (diff <= 0) * (-1 * Kf * np.multiply(low_v, [1., 1, 1]))
                b_mass.append(
                        ground_force_value + ground_friction_value + [0, -1 * G * link.mass, 0,]
                )
                b_mass.append(
                        -1 * np.cross(link.omega, np.matmul(I_world, link.omega))
                        - Kf * np.array(link.omega)
                        + np.cross(low_p - link.posn, ground_force_value + ground_friction_value)
                )
                b_cons.append(
                        weird_term + last_weird_term - Kp * (diff_velocity) - Kd * (diff_position)
                )
                last_weird_term = weird_term
        gen_mass_mat = generalizedDiag(gen_mass)
        J_mat = generalizedDiag(J, -3)
        z = np.zeros((J_mat.shape[1], J_mat.shape[1]))
        A = matrixStack([
                [gen_mass_mat, J_mat],
                [J_mat.transpose(), z]
        ])
        b = np.hstack([
                np.hstack(b_mass),
                np.hstack(b_cons)
        ])
        # print(b)
        x = np.linalg.solve(A, b)
        # print(x.reshape((-1, 3)))
        for i, link in enumerate(links):
                acc = x[i*6:i*6+3]
                omega_dot = x[i*6+3:i*6+6]
                #### explicit Euler integration to update the state
                link.posn += link.vel*dT # + acc*dT*dT/2
                link.vel += acc*dT
                link.theta += link.omega*dT # + omega_dot*dT*dT/2
                link.omega += omega_dot*dT

        simTime += dT

            #### draw the updated state
        DrawWorld()
        # printf("simTime=%.2f\n",simTime)

#####################################################
#### DrawWorld():  draw the world
#####################################################

def DrawWorld():
        global links

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	# Clear The Screen And The Depth Buffer
        glLoadIdentity();
        gluLookAt(2,2,6,  0,2,0,  0,1,0)

        DrawOrigin()
        for link in links:
                link.draw()
        # link2.draw()

        glutSwapBuffers()                      # swap the buffers to display what was just drawn

#####################################################
#### initGL():  does standard OpenGL initialization work
#####################################################

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
    glClearColor(1.0, 1.0, 0.9, 0.0)	# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)				# The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    glEnable( GL_LINE_SMOOTH );
    glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()					# Reset The Projection Matrix
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### ReSizeGLScene():    called when window is resized
#####################################################

def ReSizeGLScene(Width, Height):
    if Height == 0:						# Prevent A Divide By Zero If The Window Is Too Small 
	    Height = 1
    glViewport(0, 0, Width, Height)		# Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)    ## 45 deg horizontal field of view, aspect ratio, near, far
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### DrawOrigin():  draws RGB lines for XYZ origin of coordinate system
#####################################################

def DrawOrigin():
        glLineWidth(3.0);

        glColor3f(1,0.5,0.5)   ## light red x-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(1,0,0)
        glEnd()

        glColor3f(0.5,1,0.5)   ## light green y-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(0,1,0)
        glEnd()

        glColor3f(0.5,0.5,1)   ## light blue z-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(0,0,1)
        glEnd()

        glColor3f(0.8,0.8,0.8)   ## ground plane
        glBegin(GL_POLYGON)
        glVertex3f(1.5,0,1.5)
        glVertex3f(-1.5,0,1.5)
        glVertex3f(-1.5,0,-1.5)
        glVertex3f(1.5,0,-1.5)
        glEnd()

#####################################################
#### DrawCube():  draws a cube that spans from (-1,-1,-1) to (1,1,1)
#####################################################

def DrawCube():

	glScalef(0.5,0.5,0.5);                  # dimensions below are for a 2x2x2 cube, so scale it down by a half first
	glBegin(GL_QUADS);			# Start Drawing The Cube

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)

	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)

	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)

	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)

	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

            ### Draw the wireframe edges
	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);
     
	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

####################################################
# printf()  
####################################################

def printf(format, *args):
    sys.stdout.write(format % args)

################################################################################
# start the app

print ("Hit ESC key to quit.")
main()
