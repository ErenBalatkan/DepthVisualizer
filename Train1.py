
import glfw
import OpenGL.GL as GL

from OpenGL.GL import shaders
from OpenGL.arrays import vbo
import transformations as T

import sys
import numpy as np
import math


vertex_shader_source = \
    "#version 330 core\n" +\
    "uniform mat4 view;\n" +\
    "uniform mat4 projection;\n" +\
    "layout (location = 0) in vec3 aPos;\n" + \
    "void main()\n" +\
    "{\n" +\
    "   gl_Position = view * vec4(aPos, 1.0);\n" +\
    "}"

fragment_shader_source = \
    "#version 330 core\n" + \
    "out vec4 FragColor;\n" + \
    "void main()\n" + \
    "{\n" + \
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n" + \
    "}"


def worldCoords(self, width, height):
    cx, cy = width / 2, height / 2
    fx = 715
    fy = 715
    xx, yy = np.tile(range(width), height), np.repeat(range(height), width)
    xx = (xx - cx) / fx
    yy = (yy - cy) / fy
    return xx, yy


def perspective(fovy, aspect, n, f):
    s = 1.0/math.tan(math.radians(fovy)/2.0)
    sx, sy = s / aspect, s
    zz = (f+n)/(n-f)
    zw = 2*f*n/(n-f)
    return np.array([[sx, 0, 0, 0],
                      [0, sy, 0, 0],
                      [0, 0, zz, zw],
                      [0, 0, -1, 0]])


class DepthRenderer:
    def __init__(self):
        self.init_glfw()
        self.window = self.init_window()
        self.shader_program = self.init_shaders()

        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glPointSize(10)

        GL.glEnable(GL.GL_DEPTH_TEST)

        self.data = vbo.VBO(np.array([-0.5, -0.5, 0,
                                     0.5, -0.5, 0,
                                     0, 0.5, 0], np.float32))

        self.data.create_buffers()

        self.y_rotation = 0
        self.camera_coords = np.array([0, 0.01, 0.0], dtype=np.float32)

    def init_glfw(self):
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    def init_window(self):
        window = glfw.create_window(800, 600, "Training", None, None)
        glfw.make_context_current(window)
        GL.glViewport(0, 0, 800, 600)
        glfw.set_framebuffer_size_callback(window, self.framebuffer_size_callback)
        return window

    def framebuffer_size_callback(self, window, width, height):
        GL.glViewport(0, 0, width, height)


    def update_shader_parameters(self):
        view_loc = GL.glGetUniformLocation(self.shader_program, "view")

        view_matrix = T.translation_matrix(np.ascontiguousarray(-1 * self.camera_coords, np.float32))
        GL.glUniformMatrix4fv(view_loc, 1, GL.GL_TRUE, np.ascontiguousarray(view_matrix, np.float32))

        perspective_matrix = perspective(45, 16/9, 0.1, 100)
        # perspective_matrix = np.eye(4)

        projection_loc = GL.glGetUniformLocation(self.shader_program, "projection")
        GL.glUniformMatrix4fv(projection_loc, 1, GL.GL_TRUE, np.ascontiguousarray(perspective_matrix, np.float32))

    def render_triangle(self):
        self.camera_coords[1] += 0.01

        self.data.bind()

        self.update_shader_parameters()

        vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(vao)

        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 0, None)
        GL.glEnableVertexAttribArray(0)

        GL.glUseProgram(self.shader_program)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 9)

    def render(self):
        GL.glClearColor(0.2, 0.3, 0.3, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.render_triangle()

    def render_loop(self):
        while not glfw.window_should_close(self.window):
            self.process_input()

            self.render()

            glfw.swap_buffers(self.window)
            glfw.poll_events()

        glfw.terminate()

    def process_input(self):
        if glfw.get_key(self.window, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(self.window, True)

    def init_shaders(self):
        vertex_shader = shaders.compileShader(vertex_shader_source, GL.GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(fragment_shader_source, GL.GL_FRAGMENT_SHADER)

        shader_program = shaders.compileProgram(vertex_shader, fragment_shader)
        GL.glUseProgram(shader_program)
        return shader_program


if __name__ == '__main__':
    renderer = DepthRenderer()
    renderer.render_loop()


