import OpenGL
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import glfw
import pywavefront
import numpy
import math
from os import system
from pyquaternion import Quaternion
import keyboard

class obj():
    def __init__(self, mesh, pos, orient):
        self.mesh = mesh
        self.pos = pos
        self.orient = orient
        self.rel_orient = orient

    def get_mesh(self):
        return self.mesh

    def get_pos(self):
        return self.pos

    def set_pos(self, pos):
        self.pos = pos

    def get_orient(self):
        return self.orient

    def set_orient(self, orient):
        self.orient = orient

    def set_rel_orient(self, orient):
        self.rel_orient = orient

    def get_rel_orient(self):
        return self.rel_orient

    def rotate(self, theta_x, theta_y, theta_z):

        if theta_x:
            theta_x = math.radians(theta_x)
            rotator = Quaternion(axis=self.orient[0], angle=theta_x)
            self.set_orient(numpy.array([self.orient[0], rotator.rotate(self.orient[1]), rotator.rotate(self.orient[2])]))
            self.set_rel_orient(numpy.array([self.orient[0], rotator.rotate(self.orient[1]), rotator.rotate(self.orient[2])]))

        if theta_y:
            theta_y = math.radians(theta_y)
            rotator = Quaternion(axis=self.orient[1], angle=theta_y)
            self.set_orient(numpy.array([rotator.rotate(self.orient[0]), self.orient[1], rotator.rotate(self.orient[2])]))
            self.set_rel_orient(numpy.array([rotator.rotate(self.orient[0]), self.orient[1], rotator.rotate(self.orient[2])]))

        if theta_z:
            theta_z = math.radians(theta_z)
            rotator = Quaternion(axis=self.orient[2], angle=theta_z)
            self.set_orient(numpy.array([rotator.rotate(self.orient[0]), rotator.rotate(self.orient[1]), self.orient[2]]))
            self.set_rel_orient(numpy.array([rotator.rotate(self.orient[0]), rotator.rotate(self.orient[1]), self.orient[2]]))

    def draw(self):
        
        glPushMatrix()
        glColor(1,1,1)

        for mesh in self.mesh.mesh_list:
            glBegin(GL_TRIANGLES)
            for face in mesh.faces:
                for vertex_i in face:
                    vertex_i = self.mesh.vertices[vertex_i]
                    vertex_i = numpy.array(vertex_i)
                    vertex_i = numpy.matmul(vertex_i, self.orient)
                    vertex_i = [vertex_i[0] + self.pos[0], vertex_i[1] + self.pos[1], vertex_i[2] + self.pos[2]]
                    glVertex3f(vertex_i[0], vertex_i[1], vertex_i[2])
            glEnd()

        glBegin(GL_LINES)
        glColor(1,0,0)
        glVertex3f(self.pos[0], self.pos[1], self.pos[2])
        glVertex3f(self.orient[0][0] + self.pos[0],self.orient[0][1] + self.pos[1],self.orient[0][2] + self.pos[2])

        glColor(0,1,0)
        glVertex3f(self.pos[0], self.pos[1], self.pos[2])
        glVertex3f(self.orient[1][0] + self.pos[0],self.orient[1][1] + self.pos[1],self.orient[1][2] + self.pos[2])

        glColor(0,0,1)
        glVertex3f(self.pos[0], self.pos[1], self.pos[2])
        glVertex3f(self.orient[2][0] + self.pos[0],self.orient[2][1] + self.pos[1],self.orient[2][2] + self.pos[2])
        glEnd()

        glPopMatrix()

# ROTARY ATTACHMENT
attachment_mesh = pywavefront.Wavefront('models/attachment.obj', collect_faces=True)
attachment_pos = numpy.array([0,0,0])
attachment_orient = numpy.array([[1,0,0],[0,1,0],[0,0,1]])

attachment = obj(attachment_mesh, attachment_pos, attachment_orient)

frame_attachment_rot_y = 0
attachment_y_total = 0
attachment_y_limits = [-180, 180]
        
# SHOULDER JOINT (X and Y rotation)
shoulder_mesh = pywavefront.Wavefront('models/shoulder.obj', collect_faces=True)
shoulder_pos = numpy.array([0,0,0])
shoulder_orient = numpy.array([[1,0,0],[0,1,0],[0,0,1]])

shoulder = obj(shoulder_mesh, shoulder_pos, shoulder_orient)
shoulder_length = 4.2

frame_shoulder_rot_x = 0
shoulder_x_total = 0
shoulder_x_limits = [-90, 90] 

# ELBOW JOINT (Y rotation)
elbow_mesh = pywavefront.Wavefront('models/elbow.obj', collect_faces=True)
elbow_pos = numpy.array([0,4.2,0])
elbow_orient = numpy.array([[1,0,0],[0,1,0],[0,0,1]])

elbow = obj(elbow_mesh, elbow_pos, elbow_orient)
elbow_length = 3.7

frame_elbow_rot_y = 0
elbow_y_total = 0
elbow_y_limits = [-160, 160]

# WRIST JOINT (X, Y and Z rotation)
wrist_mesh = pywavefront.Wavefront('models/wrist.obj', collect_faces=True)
wrist_pos = numpy.array([0,7.9,0])
wrist_orient = numpy.array([[1,0,0],[0,1,0],[0,0,1]])

wrist = obj(wrist_mesh, wrist_pos, wrist_orient)

frame_wrist_rot_x = 0
frame_wrist_rot_y = 0
frame_wrist_rot_z = 0

wrist_x_total = 0
wrist_y_total = 0
wrist_z_total = 0

wrist_x_limits = [-90, 90]
wrist_y_limits = [-90, 90]
wrist_z_limits = [-180, 180]

numpy.set_printoptions(suppress=True)

def main():
    glfw.init()
    window = glfw.create_window(800, 600, "RMS Arm", None, None)
    glfw.set_window_pos(window,100,100)
    glfw.make_context_current(window)
    
    gluPerspective(60, 8/6, 0.05, 500.0)
    glEnable(GL_CULL_FACE)
    
    glTranslate(0, -4, -10)

    global frame_shoulder_rot_x, frame_attachment_rot_y,\
           frame_elbow_rot_y,\
           frame_wrist_rot_x, frame_wrist_rot_y, frame_wrist_rot_z,\
           shoulder_x_total, attachment_y_total,\
           elbow_y_total,\
           wrist_x_total, wrist_y_total, wrist_z_total

    while True:

        glfw.poll_events()

        if keyboard.is_pressed("y"):
            frame_attachment_rot_y = 1
        if keyboard.is_pressed("h"):
            frame_attachment_rot_y = -1
            
        if keyboard.is_pressed("g"):
            frame_shoulder_rot_x = 1
        if keyboard.is_pressed("j"):
            frame_shoulder_rot_x = -1

        if keyboard.is_pressed("r"):
            frame_elbow_rot_y = 1
        if keyboard.is_pressed("f"):
            frame_elbow_rot_y = -1

        if keyboard.is_pressed("w"):
            frame_wrist_rot_y = 1
        if keyboard.is_pressed("s"):
            frame_wrist_rot_y = -1
        if keyboard.is_pressed("a"):
            frame_wrist_rot_x = 1
        if keyboard.is_pressed("d"):
            frame_wrist_rot_x = -1
        if keyboard.is_pressed("q"):
            frame_wrist_rot_z = 1
        if keyboard.is_pressed("e"):
            frame_wrist_rot_z = -1

        attachment_y_total += frame_attachment_rot_y

        shoulder_x_total += frame_shoulder_rot_x

        elbow_y_total += frame_elbow_rot_y

        wrist_x_total += frame_wrist_rot_x
        wrist_y_total += frame_wrist_rot_y
        wrist_z_total += frame_wrist_rot_z

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        # move, rotate and draw models
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

        attachment.rotate(0, frame_attachment_rot_y, 0)
        attachment.draw()

        # rotate shoulder with attachment on y axis
        rotator = Quaternion(axis=attachment.get_orient()[1], angle=math.radians(frame_attachment_rot_y))
        shoulder.set_orient(numpy.array([rotator.rotate(shoulder.orient[0]), rotator.rotate(shoulder.orient[1]), rotator.rotate(shoulder.orient[2])]))
        
        shoulder.rotate(frame_shoulder_rot_x, 0, 0)
        shoulder.draw()

        # elbow rotates with both attachment point and shoulder
        rotator = Quaternion(axis=attachment.get_orient()[1], angle=math.radians(frame_attachment_rot_y))
        elbow.set_orient(numpy.array([rotator.rotate(elbow.orient[0]), rotator.rotate(elbow.orient[1]), rotator.rotate(elbow.orient[2])]))
        
        rotator = Quaternion(axis=shoulder.get_orient()[0], angle=math.radians(frame_shoulder_rot_x))
        elbow.set_orient(numpy.array([rotator.rotate(elbow.orient[0]), rotator.rotate(elbow.orient[1]), rotator.rotate(elbow.orient[2])]))

        elbow.set_pos(numpy.matmul(numpy.array([0, shoulder_length, 0]), shoulder.get_orient()))

        elbow.rotate(frame_elbow_rot_y, 0, 0)
        elbow.draw()

        # wrist rotates with attachment point, shoulder and elbow
        rotator = Quaternion(axis=attachment.get_orient()[1], angle=math.radians(frame_attachment_rot_y))
        wrist.set_orient(numpy.array([rotator.rotate(wrist.orient[0]), rotator.rotate(wrist.orient[1]), rotator.rotate(wrist.orient[2])]))
        
        rotator = Quaternion(axis=shoulder.get_orient()[0], angle=math.radians(frame_shoulder_rot_x))
        wrist.set_orient(numpy.array([rotator.rotate(wrist.orient[0]), rotator.rotate(wrist.orient[1]), rotator.rotate(wrist.orient[2])]))

        rotator = Quaternion(axis=elbow.get_orient()[0], angle=math.radians(frame_elbow_rot_y))
        wrist.set_orient(numpy.array([rotator.rotate(wrist.orient[0]), rotator.rotate(wrist.orient[1]), rotator.rotate(wrist.orient[2])]))
        
        wrist.set_pos(elbow.get_pos() + numpy.matmul(numpy.array([0, elbow_length, 0]), elbow.get_orient()))

        wrist.rotate(frame_wrist_rot_x, frame_wrist_rot_y, frame_wrist_rot_z)
        wrist.draw()

        # clear rotation commands
        frame_attachment_rot_y = 0

        frame_shoulder_rot_x = 0

        frame_elbow_rot_y = 0

        frame_wrist_rot_x = 0
        frame_wrist_rot_y = 0
        frame_wrist_rot_z = 0

        glfw.swap_buffers(window)

main()
