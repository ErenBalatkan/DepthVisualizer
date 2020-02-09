import glfw
import OpenGL.GL as GL
from OpenGL.GL import shaders
from OpenGL.arrays import vbo
import math
import glm
import cv2
import ctypes
import time
from PIL import Image
import cupy as cp
import numpy as np

FRAME_WIDTH, FRAME_HEIGHT = 1600, 1200

vertex_shader_source = \
    "#version 330 core\n" + \
    "uniform mat4 view;\n" + \
    "uniform mat4 projection;\n" + \
    "layout (location = 0) in vec3 aPos;\n" + \
    "layout (location = 1) in vec3 aColor;\n" + \
    "out vec3 vertexColor;\n" + \
    "void main()\n" + \
    "{\n" + \
    "   gl_Position = projection * view * vec4(aPos, 1.0);\n" + \
    "   vertexColor = aColor;" + \
    "}"

fragment_shader_source = \
    "#version 330 core\n" + \
    "out vec4 FragColor;\n" + \
    "in vec3 vertexColor;\n" + \
    "void main()\n" + \
    "{\n" + \
    "   FragColor = vec4(vertexColor, 1);\n" + \
    "}"


class DepthRenderer:
    def __init__(self, frame_width, frame_height, window_name="Depth Visualizer",
                 camera_move_speed=20, camera_turn_speed=90, point_size=3):
        self.frame_width, self.frame_height = frame_width, frame_height
        self.window_name = window_name
        self.camera_move_speed, self.camera_turn_speed = camera_move_speed, camera_turn_speed
        self.point_size = point_size
        self.last_render_time = time.time()

        self.init_glfw()
        self.__window = self.init_window()
        self.__shader_program = self.init_shaders()
        self.__vao_pointer = self.init_opengl_vars()

        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glPointSize(self.point_size)
        GL.glEnable(GL.GL_DEPTH_TEST)

        self.__point_data = vbo.VBO(np.array([], np.float32))
        self.__point_data.create_buffers()

        self.__line_data = vbo.VBO(np.array([], np.float32))
        self.__line_data.create_buffers()

        self.__voxel_data = vbo.VBO(np.array([], np.float32))
        self.__voxel_data.create_buffers()
        self.__voxel_indicies = np.array([0, 1, 2,
                                          0, 3, 2,
                                          4, 5, 6,
                                          4, 7, 6,
                                          0, 3, 7,
                                          1, 5, 6,
                                          1, 2, 6,
                                          0, 4, 5,
                                          0, 1, 5,
                                          3, 7, 6,
                                          3, 2, 6
                                          ], np.int32)
        self.__voxel_indicies_data = vbo.VBO(self.__voxel_indicies, target=GL.GL_ELEMENT_ARRAY_BUFFER)
        self.__voxel_indicies_data.create_buffers()

        self.__camera_coords = glm.vec3(0, 10, 10)
        self.__camera_front = glm.vec3(0, 0, -1)
        self.__camera_up = glm.vec3(0, 1, 0)

        self.camera_yaw = -90
        self.camera_pitch = -20

        self.__mouse_pos = [FRAME_WIDTH / 2, FRAME_HEIGHT / 2]

        self.update_shader_parameters()
        self.change_camera_rotation(0, 0)

    def init_glfw(self):
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    def hide_window(self):
        glfw.hide_window(self.__window)

    def show_window(self):
        glfw.show_window(self.__window)

    def init_window(self):
        window = glfw.create_window(FRAME_WIDTH, FRAME_HEIGHT, self.window_name, None, None)
        glfw.make_context_current(window)
        GL.glViewport(0, 0, FRAME_WIDTH, FRAME_HEIGHT)
        glfw.set_framebuffer_size_callback(window, self.framebuffer_size_callback)
        return window

    def framebuffer_size_callback(self, window, width, height):
        GL.glViewport(0, 0, width, height)

    @staticmethod
    def convert_depth_pixel_to_point(x, y, depth, camera_resolution, focal_length_in_pixels, rgb=[255, 255, 255]):
        camera_resolution = np.array(camera_resolution)
        view_center = camera_resolution / 2
        x_dist = view_center[0] - x
        y_dist = view_center[1] - y
        pixel_dist = (x_dist ** 2 + y_dist ** 2) ** 0.5
        focal_target = (focal_length_in_pixels ** 2 + pixel_dist ** 2) ** 0.5

        z = depth * focal_length_in_pixels / focal_target
        x = x_dist / focal_length_in_pixels * z
        y = y_dist / focal_length_in_pixels * z

        return [x, y, z] + rgb

    def update_shader_parameters(self):
        view_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "view")
        view_matrix = glm.lookAt(self.__camera_coords, self.__camera_coords + self.__camera_front, self.__camera_up)
        GL.glUniformMatrix4fv(view_matrix_loc, 1, GL.GL_FALSE, np.ascontiguousarray(view_matrix, np.float32))

        perspective_matrix = glm.perspective(glm.radians(30), FRAME_WIDTH / FRAME_HEIGHT, 0.1, 200)

        projection_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "projection")
        GL.glUniformMatrix4fv(projection_matrix_loc, 1, GL.GL_FALSE,
                              np.ascontiguousarray(perspective_matrix, np.float32))

    def handle_vertex_attrib(self):
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(self.__point_data.data.itemsize * 3))
        GL.glEnableVertexAttribArray(1)

    def render_points(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_POINT)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glUseProgram(self.__shader_program)

        self.__point_data.bind()
        self.handle_vertex_attrib()
        GL.glDrawArrays(GL.GL_POINTS, 0, len(self.__point_data.data))

        GL.glBindVertexArray(0)

    def render_lines(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
        GL.glBindVertexArray(self.__vao_pointer)
        self.__line_data.bind()

        self.handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawArrays(GL.GL_LINES, 0, len(self.__line_data.data))
        GL.glBindVertexArray(0)

    def render_voxels(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
        GL.glBindVertexArray(self.__vao_pointer)

        self.__voxel_indicies_data.bind()
        self.__voxel_data.bind()

        self.handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawElements(GL.GL_TRIANGLES, len(self.__voxel_data.data), GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(0)

    def convert_depthmap_to_points(self, depth_map, focal_length, rgb_map=None):
        if depth_map.shape[0] == 1:
            depth_map = np.swapaxes(depth_map, 0, 1).swapaxes(1, 2)

        points = np.ones(shape=(depth_map.shape[0] * depth_map.shape[1], 6))  # A point contains x,y,z,r,g,b
        if rgb_map is None:
            points[:, 3:6] = [0.5, 0.7, 1]
        else:
            if rgb_map.shape[0] == 3:
                rgb_map = np.swapaxes(rgb_map, 0, 1).swapaxes(1, 2)
            rgb_map = rgb_map.reshape(-1, 3)
            points[:, 3:6] = rgb_map / 255.0

        y, x = np.meshgrid(np.arange(0, depth_map.shape[1]), np.arange(0, depth_map.shape[0]))
        yx_coordinates = np.array([x.flatten(), y.flatten()], np.float32).T
        yx_coordinates[:] += -1 * np.array(depth_map.shape[:2]) / 2
        yx_coordinates = np.flip(yx_coordinates, 1)
        points[:, 0:2] = yx_coordinates
        points[:, 2] = depth_map.flatten()
        points = points[points[:, 2] >= 5]

        pixel_dist = (points[:, 0] ** 2 + points[:, 1] ** 2) ** 0.5
        focal_target_dist = (focal_length ** 2 + pixel_dist ** 2) ** 0.5
        # points[:, 2] = points[:, 2] * focal_length / focal_target_dist
        points[:, 0] = points[:, 0] * points[:, 2] / focal_length
        points[:, 1] = points[:, 1] * points[:, 2] / focal_length

        points[:, 1] *= -1
        return points

    def convert_point_data_z_axis(self, point_data):
        converted_data = np.asarray(point_data, np.float32).reshape(-1, 6)
        converted_data[:, 2] *= -1
        converted_data = converted_data.flatten()
        return converted_data

    def set_points(self, points):
        points = np.array(points, np.float32)
        self.reset_points()
        self.add_points(points)

    def add_points(self, points):
        points = np.array(points, np.float32)
        old_points = self.__point_data.data

        points = self.convert_point_data_z_axis(points)
        if len(old_points) == 0:
            new_points = points
        else:
            new_points = np.append(old_points, points, axis=0)
        self.__point_data.set_array(new_points)

    def reset_points(self):
        self.__point_data.set_array(np.array([], np.float32))

    def set_lines(self, lines):
        self.reset_lines()
        self.add_lines(lines)

    def add_lines(self, lines):
        old_lines = self.__line_data.data

        if len(lines.shape) > 1:
            lines = lines.flatten()

        lines = self.convert_point_data_z_axis(lines)
        if len(old_lines) == 0:
            new_lines = lines
        else:
            new_lines = np.append(old_lines, lines, axis=0)
        self.__line_data.set_array(new_lines)

    def reset_lines(self):
        self.__line_data.set_array(np.array([], np.float32))
        self.__voxel_indicies_data.set_array(np.array([], np.int32))

    def set_voxels(self, voxels):
        self.reset_voxels()
        self.add_voxels(voxels)

    def __convert_voxels_to_opengl_format(self, voxels):
        voxels = np.array(voxels, np.float32)
        voxels = voxels.reshape(-1, 7)
        formatted_voxels = np.zeros(shape=(len(voxels), 6*8))

        for i in range(8):
            formatted_voxels[:, i*6+3:i*6+6] = voxels[:, 4:7]

        size = voxels[:, 3] / 2
        formatted_voxels[:, 0] = voxels[:, 0] - size  # x0
        formatted_voxels[:, 1] = voxels[:, 1] + size  # y0
        formatted_voxels[:, 2] = voxels[:, 2] - size  # z0

        formatted_voxels[:, 0 + 6] = voxels[:, 0] + size  # x1
        formatted_voxels[:, 1 + 6] = voxels[:, 1] + size  # y1
        formatted_voxels[:, 2 + 6] = voxels[:, 2] - size  # z1

        formatted_voxels[:, 0 + 6*2] = voxels[:, 0] + size  # x2
        formatted_voxels[:, 1 + 6*2] = voxels[:, 1] - size  # y2
        formatted_voxels[:, 2 + 6*2] = voxels[:, 2] - size  # z2

        formatted_voxels[:, 0 + 6*3] = voxels[:, 0] - size  # x3
        formatted_voxels[:, 1 + 6*3] = voxels[:, 1] - size  # y3
        formatted_voxels[:, 2 + 6*3] = voxels[:, 2] - size  # z3

        formatted_voxels[:, 0 + 6*4] = voxels[:, 0] - size  # x4
        formatted_voxels[:, 1 + 6*4] = voxels[:, 1] + size  # y4
        formatted_voxels[:, 2 + 6*4] = voxels[:, 2] + size  # z4

        formatted_voxels[:, 0 + 6*5] = voxels[:, 0] + size  # x5
        formatted_voxels[:, 1 + 6*5] = voxels[:, 1] + size  # y5
        formatted_voxels[:, 2 + 6*5] = voxels[:, 2] + size  # z5

        formatted_voxels[:, 0 + 6*6] = voxels[:, 0] + size  # x6
        formatted_voxels[:, 1 + 6*6] = voxels[:, 1] - size  # y6
        formatted_voxels[:, 2 + 6*6] = voxels[:, 2] + size  # z6

        formatted_voxels[:, 0 + 6*7] = voxels[:, 0] - size  # x7
        formatted_voxels[:, 1 + 6*7] = voxels[:, 1] - size  # y7
        formatted_voxels[:, 2 + 6*7] = voxels[:, 2] + size  # z7
        return formatted_voxels.flatten()

    def add_voxels(self, voxels):
        '''

        :param voxels: [x, y, z, size, r, g b]
        :return:
        '''
        voxels = self.__convert_voxels_to_opengl_format(voxels)
        old_voxels = self.__voxel_data.data

        voxels = self.convert_point_data_z_axis(voxels)
        if len(old_voxels) == 0:
            new_voxels = voxels
        else:
            new_voxels = np.append(old_voxels, voxels, axis=0)

        indicie_repeat_count = int(len(new_voxels) / 8.0)
        indicie_offset = (np.array(range(indicie_repeat_count)) * 8).reshape(-1, 1)
        new_indicies = np.tile(self.__voxel_indicies, indicie_repeat_count).reshape(-1, len(
            self.__voxel_indicies)) + indicie_offset
        new_indicies = np.asarray(new_indicies, np.int32)

        self.__voxel_indicies_data.set_array(new_indicies.flatten())
        self.__voxel_data.set_array(new_voxels)

    def reset_voxels(self):
        self.__voxel_data.set_array(np.array([], np.float32))

    def add_quad(self, quad_data, camera_resolution, focal_length_in_pixels=300, color=[255, 255, 255], depth=10,
                 draw_3d=False, max_depth=80):
        """ Draws a 2d quad, distance between camera and quad is specified by depth parameter, if draw3d
        parameter is set to true, will draw 2nd quad of same shape max_depth meters away from camera and will connect
        corners of first and second quad with lines
        :param quad_data: A 4x2 matrix that contains pixel locations [[pixel1_x, pixel1_y], [pixel2_x, pixel2_y], ...]
        :param camera_resolution: Resolution of the camera that recorded the image in which quad resides
        :param focal_length_in_pixels: Camera's focal length represent in pixels, refer to following link to calculate
        https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/?answer=17180#post-id-17180
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :param depth: Distance from camera in which quad will be drawn
        :param draw_3d: Draws the quad as a combination of 2 quad located at different depths, max_depth argument
        specifies the location of the 2nd quad
        :param max_depth: Distance from camera in which 2nd quad will be drawn
        :return:
        """
        first_quad_points = np.array([self.convert_depth_pixel_to_point(x, y, depth, camera_resolution,
                                                                        focal_length_in_pixels, color)
                                      for x, y in quad_data])

        self.add_lines(np.array([first_quad_points[0], first_quad_points[1], first_quad_points[1], first_quad_points[2],
                                 first_quad_points[2], first_quad_points[3], first_quad_points[3],
                                 first_quad_points[0]]).flatten())

        if draw_3d:
            second_quad_points = np.array([self.convert_depth_pixel_to_point(x, y, max_depth, camera_resolution,
                                                                             focal_length_in_pixels, color)
                                           for x, y in quad_data])
            self.add_lines(
                np.array([second_quad_points[0], second_quad_points[1], second_quad_points[1], second_quad_points[2],
                          second_quad_points[2], second_quad_points[3], second_quad_points[3],
                          second_quad_points[0]]).flatten())

            self.add_lines(np.array([[first_quad_points[x], second_quad_points[x]] for x in range(4)]).flatten())

    def add_3d_bbox(self, bbox_data, color=[255, 255, 255]):
        """ Draws 3d bounding box
        :param bbox_data: Represented in format [center_x, center_y, center_z, width, height, length, y_rotation]
        (needs orientation aswell, put on hold)
        :param camera_resolution:
        :param focal_length_in_pixels:
        :param color:
        :return:
        """
        color = np.array(color) / 255.0
        first_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2],
                               [bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2]]

        second_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2]]

        rotation_matrix = glm.translate(glm.mat4(1), glm.vec3(*bbox_data[:3]))
        rotation_matrix = glm.rotate(rotation_matrix, bbox_data[-1], glm.vec3(0, 1, 0))

        first_rect_centered = [list(rotation_matrix * glm.vec4(*vec, 1))[:3] for vec in first_rect_centered]
        second_rect_centered = [list(rotation_matrix * glm.vec4(*vec, 1))[:3] for vec in second_rect_centered]

        lines = [first_rect_centered[0], first_rect_centered[1],
                 first_rect_centered[1], first_rect_centered[2],
                 first_rect_centered[2], first_rect_centered[3],
                 first_rect_centered[3], first_rect_centered[0],
                 second_rect_centered[0], second_rect_centered[1],
                 second_rect_centered[1], second_rect_centered[2],
                 second_rect_centered[2], second_rect_centered[3],
                 second_rect_centered[3], second_rect_centered[0]
                 ]
        lines = np.append(lines, np.array([[first_rect_centered[x], second_rect_centered[x]] for x
                                           in range(4)]).reshape(-1, 3), axis=0)
        lines = np.array([[x, y, z, color[0], color[1], color[2]] for x, y, z in lines])
        self.add_lines(lines)

    def set_camera_coords(self, x, y, z):
        self.__camera_coords = glm.vec3(x, y, -z)

    def move_forward(self, meters):
        self.__camera_coords += self.__camera_front * meters

    def move_right(self, meters):
        self.__camera_coords += glm.normalize(glm.cross(self.__camera_front, self.__camera_up)) * meters

    def look_up(self, angles):
        self.change_camera_rotation(0, angles)

    def look_right(self, angles):
        self.change_camera_rotation(angles, 0)

    def process_input(self, frame_delay=0.016, process_movement=True):
        if glfw.get_key(self.__window, glfw.KEY_ESCAPE) == glfw.PRESS:
            glfw.set_window_should_close(self.__window, True)
        if not process_movement:
            return
        if glfw.get_key(self.__window, glfw.KEY_W) == glfw.PRESS:
            self.move_forward(self.camera_move_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_S) == glfw.PRESS:
            self.move_forward(-1 * self.camera_move_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_A) == glfw.PRESS:
            self.move_right(-1 * self.camera_move_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_D) == glfw.PRESS:
            self.move_right(self.camera_move_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_UP) == glfw.PRESS:
            self.look_up(self.camera_turn_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_DOWN) == glfw.PRESS:
            self.look_up(-1 * self.camera_turn_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_RIGHT) == glfw.PRESS:
            self.look_right(self.camera_turn_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_LEFT) == glfw.PRESS:
            self.look_right(-1 * self.camera_turn_speed * frame_delay)
        if glfw.get_key(self.__window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS:
            self.__camera_coords[1] += self.camera_move_speed * frame_delay
        if glfw.get_key(self.__window, glfw.KEY_LEFT_CONTROL) == glfw.PRESS:
            self.__camera_coords[1] -= self.camera_move_speed * frame_delay

    def add_voxel_map(self, voxel_map, voxel_map_center, voxel_size, filter_black_voxels=True):
        ot = time.time()
        x, y, z = np.meshgrid(np.arange(0, voxel_map.shape[0]), np.arange(0, voxel_map.shape[1]), np.arange(0, voxel_map.shape[2]), indexing='ij')
        xyz_coordinates = np.array([x.flatten(), y.flatten(), z.flatten()], np.float32).T
        xyz_coordinates[:, 0] -= voxel_map.shape[0] / 2
        xyz_coordinates[:, 1] -= voxel_map.shape[1] / 2
        xyz_coordinates[:, 2] -= voxel_map.shape[2] / 2

        print(voxel_map.shape)

        print("axmt", time.time() - ot)
        ot = time.time()

        xyz_coordinates *= voxel_size
        xyz_coordinates += voxel_size / 2

        xyz_coordinates += voxel_map_center

        print("axmt", time.time() - ot)
        ot = time.time()

        voxel_data = np.zeros(shape=(voxel_map.shape[0] * voxel_map.shape[1] * voxel_map.shape[2], 7))
        flattened_map = voxel_map.reshape(-1, 3)
        voxel_data[:, :3] = xyz_coordinates
        voxel_data[:, 3] = voxel_size
        voxel_data[:, 4:] = flattened_map

        print("axmt", time.time()-ot)
        ot = time.time()
        if filter_black_voxels:
            voxel_data = voxel_data[voxel_data[:, 4] != 0]

        self.add_voxels(voxel_data.flatten())

    def convert_points_to_voxel_map(self, points, voxel_map_center, voxel_map_size, voxel_size):
        '''

        :param points: [[x1, y1, z1, r, g, b]]
        :param voxel_map_center: [x, y, z]
        :param voxel_map_size: [width, height, length]
        :param voxel_size: float
        :return:
        '''
        voxel_map_size = np.asarray(np.ceil(np.array(voxel_map_size)/voxel_size), np.int32)
        center_x, center_y, center_z = voxel_map_center

        x_begin, x_end = [center_x + sign * 0.5 * voxel_map_size[0] * voxel_size for sign in [-1, 1]]
        y_begin, y_end = [center_y + sign * 0.5 * voxel_map_size[1] * voxel_size for sign in [-1, 1]]
        z_begin, z_end = [center_z + sign * 0.5 * voxel_map_size[2] * voxel_size for sign in [-1, 1]]

        voxel_map = np.zeros(shape=(*voxel_map_size, 3))
        for point in points:
            x, y, z, r, g, b = point

            if x_begin < x < x_end and y_begin < y < y_end and z_begin < z < z_end:
                voxel_map[math.floor((x-x_begin)/voxel_size), math.floor((y-y_begin)/voxel_size),
                          math.floor((z-z_begin)/voxel_size)] = \
                    [r, g, b]

        return voxel_map

    def render(self, enable_controls=False):
        current_time = time.time()
        self.process_input(current_time - self.last_render_time, enable_controls)
        self.last_render_time = current_time

        GL.glClearColor(0.2, 0.3, 0.3, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.update_shader_parameters()
        self.render_points()
        self.render_lines()
        self.render_voxels()
        glfw.swap_buffers(self.__window)
        glfw.poll_events()

    def should_continue_rendering(self):
        return not glfw.window_should_close(self.__window)

    def get_rendered_frame(self):
        GL.glReadBuffer(GL.GL_BACK)
        data = GL.glReadPixels(0, 0, FRAME_WIDTH, FRAME_HEIGHT, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        image = np.frombuffer(data, dtype=np.uint8).reshape(FRAME_HEIGHT, FRAME_WIDTH, 3)
        image = np.flip(image, 0)
        return image

    def render_loop(self, enable_controls=True):
        while self.should_continue_rendering():
            self.render(enable_controls)

    def close(self):
        glfw.terminate()

    def change_camera_rotation(self, yaw_offset, pitch_offset):
        self.camera_yaw += yaw_offset
        self.camera_pitch += pitch_offset
        self.camera_pitch = np.clip(self.camera_pitch, -89, 89)
        direction = glm.vec3()
        direction.x = math.cos(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        direction.y = math.sin(glm.radians(self.camera_pitch))
        direction.z = math.sin(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        self.__camera_front = glm.normalize(direction)

    def mouse_callback(self, window, xpos, ypos):
        old_xpos, old_ypos = self.__mouse_pos
        x_offset = xpos - old_xpos
        y_offset = old_ypos - ypos
        self.__mouse_pos = [xpos, ypos]

        x_offset, y_offset = [x * self.camera_turn_speed for x in (x_offset, y_offset)]
        self.change_camera_rotation(x_offset, y_offset)

    def init_shaders(self):
        vertex_shader = shaders.compileShader(vertex_shader_source, GL.GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(fragment_shader_source, GL.GL_FRAGMENT_SHADER)

        shader_program = shaders.compileProgram(vertex_shader, fragment_shader)
        GL.glUseProgram(shader_program)
        return shader_program

    def init_opengl_vars(self):
        vao = GL.glGenVertexArrays(1)
        return vao


if __name__ == '__main__':
    renderer = DepthRenderer(FRAME_WIDTH, FRAME_HEIGHT)
    lines = [[-10, 0, 0, 0, 0.8, 0], [10, 0, 0, 1, 1, 1], [0, 10, 0, 1, 1, 1], [0, -10, 0, 0.8, 0, 0],
             [0, 0, -10, 0, 0, 0.8], [0, 0, 10, 1, 1, 1]]
    lines = np.array(lines, np.float32).flatten()
    renderer.set_lines(lines)

    renderer.add_quad([[10, 20], [50, 20], [50, 60], [10, 60]], [500, 300], 715, [0, 0, 255], 10, True, 15)

    # renderer.set_voxels([2.5,  10, 2.5, 1, 1, 1,
    #                      10, 10, 2.5, 1, 1, 1,
    #                      10, 2.5,  2.5, 1, 1, 1,
    #                      2.5,  2.5,  2.5, 1, 1, 1,
    #                      2.5,  10, 10, 1, 1, 1,
    #                      10, 10, 10, 1, 1, 1,
    #                      10, 2.5,  10, 1, 1, 1,
    #                      2.5,  2.5,  10, 1, 1, 1,
    #                      -2.5, 10, 2.5, 1, 1, 1,
    #                      -10, 10, 2.5, 1, 1, 1,
    #                      -10, 2.5, 2.5, 1, 1, 1,
    #                      -2.5, 2.5, 2.5, 1, 1, 1,
    #                      -2.5, 10, 10, 1, 1, 1,
    #                      -10, 10, 10, 1, 1, 1,
    #                      -10, 2.5, 10, 1, 1, 1,
    #                      -2.5, 2.5, 10, 1, 1, 1,
    #                      ])

    depth_map = np.asarray(Image.open("depth.png"), np.float32)
    depth_map = np.expand_dims(depth_map, axis=2) / 256.0

    rgb = cv2.cvtColor(cv2.imread("ip.png"), cv2.COLOR_BGR2RGB)

    points = renderer.convert_depthmap_to_points(depth_map, 715, rgb)
    renderer.add_points(points)
    import time
    old_t = time.time()

    renderer.add_3d_bbox([0.4, 0, 16, 2, 2, 4, 0])
    voxel_map = renderer.convert_points_to_voxel_map(points, [0.4, 0, 16], [2, 2, 4], 0.10)
    renderer.add_voxel_map(voxel_map, [0, 0, -20], 0.10)

    print("tp", time.time() - old_t)
    renderer.add_3d_bbox([5, 10, 5, 5, 5, 5, 0], color=[0, 255, 0])
    renderer.render_loop()
