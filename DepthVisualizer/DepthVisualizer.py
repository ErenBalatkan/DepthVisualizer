<<<<<<< HEAD
import glfw
import OpenGL.GL as GL
from OpenGL.GL import shaders
from OpenGL.arrays import vbo
import math
import glm
import ctypes
import time
import numpy as np
from PIL import Image

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


class Utils:
    def __init__(self):
        pass

    @staticmethod
    def convert_depth_pixel_to_point(x, y, depth, focal_length_in_pixels, principal_point
                                     , rgb=[255, 255, 255], is_depth_along_z=True):
        '''
        This method is used for converting a single pixel to 3D point
        :param x: X coordinate of pixel in image plane
        :param y: Y coordinate of the pixel in image plane
        :param depth: Depth of the pixel
        :param focal_length_in_pixels: Focal length of the camera measured in pixels
        :param principal_point: Center of the image
        :param rgb: Color of the pixel
        :param is_depth_along_z: If True, z coordinate of the pixel will be equal to depth, otherwise depth will be
        calculated as distance between camera and pixel
        :return: A point of form [x, y, z, r, g, b]
        '''
        rgb = list(rgb)

        x_dist = x - principal_point[1]
        y_dist = y - principal_point[0]

        z = depth
        if not is_depth_along_z:
            pixel_dist = (x_dist ** 2 + y_dist ** 2) ** 0.5
            focal_target = (focal_length_in_pixels ** 2 + pixel_dist ** 2) ** 0.5
            z = depth * focal_length_in_pixels / focal_target

        x = x_dist / focal_length_in_pixels * z
        y = y_dist / focal_length_in_pixels * z

        return [x, -y, z] + rgb

    @staticmethod
    def convert_point_to_pixel(point, focal_length_in_pixels, principal_point):
        '''
        This method is used for converting a point in [x, y, z, r, g, b] to a RGBD pixel
        :param point: A point of form [x, y, z, r, g, b]
        :param focal_length_in_pixels: Focal length of the camera measured in pixels
        :param principal_point: Center of the image
        :return: An RGBD pixel in shape [y, x, depth, r, g, b]
        '''

        x_dist = point[0] / point[2] * focal_length_in_pixels
        y_dist = point[1] / point[2] * focal_length_in_pixels

        y_coord = principal_point[0] - y_dist
        x_coord = principal_point[1] + x_dist

        return [y_coord, x_coord] + point[2:]

    @staticmethod
    def read_kitti_calibration(path):
        '''
        This method reads Kitti calibration file and converts it to a dictionary
        :param path: Path for the kitti dataset calibration txt
        :return: Dictionary containing calibration information
        '''
        calib_data = {}
        with open(path, 'r') as f:
            for line in f.readlines():
                if ':' in line:
                    key, value = line.split(":", 1)
                    calib_data[key] = np.array([float(x) for x in value.split()])

        R0 = calib_data['P0'].reshape(3, 4)
        focal_length = R0[1, 1]
        principal_point = R0[1, 2], R0[0, 2]

        calib_data["focal_length"] = focal_length
        calib_data["principal_point"] = principal_point

        return calib_data

    @staticmethod
    def convert_depthmap_to_points(depth_map, focal_length_in_pixels=None, principal_point=None, rgb_image=None,
                                   is_depth_along_z=True):
        '''
        Converts given depth map to point cloud
        :param depth_map: A grayscale image where values are equal to depth in meters
        :param focal_length_in_pixels: Focal lenght of the camera measured in pixels
        :param principal_point: Center of image
        :param rgb_image: Colored image that matches depth map, used for coloring points
        :param is_depth_along_z: If True, z coordinate of the pixel will be equal to depth, otherwise depth will be
        calculated as distance between camera and pixel
        :return: List of points where each point is of form [x, y, z, r, g, b]
        '''
        if focal_length_in_pixels is None:
            focal_length_in_pixels = 715
        if principal_point is None:
            principal_point = [depth_map.shape[0] / 2, depth_map.shape[1] / 2]

        if depth_map.shape[0] == 1:
            depth_map = np.swapaxes(depth_map, 0, 1).swapaxes(1, 2)

        points = np.ones(shape=(depth_map.shape[0] * depth_map.shape[1], 6))  # A point contains x,y,z,r,g,b
        if rgb_image is None:
            points[:, 3:6] = [0.5, 0.7, 1]
        else:
            if rgb_image.shape[0] == 3:
                rgb_image = np.swapaxes(rgb_image, 0, 1).swapaxes(1, 2)
            rgb_image = rgb_image.reshape(-1, 3)
            points[:, 3:6] = rgb_image / 256.0

        y, x = np.meshgrid(np.arange(0, depth_map.shape[1]), np.arange(0, depth_map.shape[0]))
        yx_coordinates = np.array([x.flatten(), y.flatten()], np.float32).T
        yx_coordinates += -1 * np.array(principal_point)
        yx_coordinates = np.flip(yx_coordinates, 1)

        points[:, 0:2] = yx_coordinates
        points[:, 2] = depth_map.flatten()

        pixel_dist = (points[:, 0] ** 2 + points[:, 1] ** 2) ** 0.5
        focal_target_dist = (focal_length_in_pixels ** 2 + pixel_dist ** 2) ** 0.5

        if not is_depth_along_z:
            points[:, 2] = points[:, 2] * focal_length_in_pixels / focal_target_dist

        points[:, 0] = points[:, 0] * points[:, 2] / focal_length_in_pixels
        points[:, 1] = points[:, 1] * points[:, 2] / focal_length_in_pixels

        points[:, 1] *= -1
        return points

    @staticmethod
    def read_kitti_point_cloud(path, calib, color=[255, 255, 255]):
        '''
        Reads kitti point cloud file and converts its coordinate space from velodyne to camera format
        :param path: Path of the point cloud file
        :param calib: Calibration dictionary, can be read from file using read_kitti_calibration method
        :param color: Color of points
        :return: Returns a list of points where each point is of format [x, y, z, r, g, b]
        '''
        color = np.array(color) / 256.0
        points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
        points = points[:, :3]
        new_points = np.ones(shape=(len(points), 4))
        new_points[:, :3] = points

        Tr_velo_to_cam = np.zeros((4, 4))
        Tr_velo_to_cam[3, 3] = 1
        Tr_velo_to_cam[:3, :4] = calib['Tr_velo_to_cam'].reshape(3, 4)

        converted_points = Tr_velo_to_cam.dot(new_points.T).T

        point_cloud = np.zeros((points.shape[0], 6))
        point_cloud[:, :3] = converted_points[:, :3]
        point_cloud[:, 1] *= -1

        point_cloud[:, 3:] = color
        return point_cloud

    @staticmethod
    def read_depth_map(path):
        depth_map = np.asarray(Image.open(path), np.float32)
        depth_map = np.expand_dims(depth_map, axis=2) / 256.0
        return depth_map

    @staticmethod
    def convert_objects_from_kitti_format(objects):
        '''
        Takes a list of objects that is on the kitti format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D Width, 3D Height,
        3D Length, 3D X, 3D Y, 3D Z, 3D Rotation]
        and converts them to DepthVisualizer format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        :param objects: List of objects where each object is on the kitti format
        :return: List of objects where each object is on the DepthVisualizer format
        '''
        converted_objects = []
        for object in objects:
            object = object.copy()
            object_data_array = np.array([float(x) for x in object[8:]], np.float32)
            object[8 + 0:8 + 3] = object_data_array[0 + 3:3 + 3]
            object[8 + 1] *= -1

            object[11:14] = object_data_array[0:3]

            object[11], object[12] = object[12], object[11]

            object[8 + 1] += object[12] / 2

            object[14] += math.radians(90)

            converted_objects.append(object)
        return converted_objects

    @staticmethod
    def convert_objects_to_kitti_format(objects):
        '''
        Takes a list of objects that is on the DepthVisualizer format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        and converts them to Kitti format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D Width, 3D Height,
        3D Length, 3D X, 3D Y, 3D Z, 3D Rotation]
        :param objects: List of objects where each object is on the DepthVisualizer format
        :return: List of objects where each object is on the Kitti format
        '''
        converted_objects = []

        for object in objects.copy():
            object_clone = object.copy()

            object_clone[8 + 1] -= object[12] / 2
            object_clone[8 + 1] *= -1

            object[11], object[12] = object[12], object[11]

            object_clone[11:14] = object_clone[8:11]
            object_clone[8:11] = object[11:14]

            object_clone[14] -= math.radians(90)

            converted_objects.append(object_clone)

        return converted_objects

    @staticmethod
    def convert_points_to_voxel_map(points, voxel_map_center, voxel_map_size, voxel_size):
        '''
        Converts points that are inside specified 3D box into voxels
        :param points: List of points where each point has following format [x, y, z, r, g, b]
        :param voxel_map_center: Center coordinates of the area that will be voxellized in format [x, y, z]
        :param voxel_map_size: Size of the area that will be voxellized in format [width, height, length]
        :param voxel_size: Size of the each voxel
        :return: A 3D array in x-y-z format where each element is list of size 7 where each element is of following
        format [r , g, b, mean x, mean y, mean z, point_count]. point_count is equal to number of points that are
        inside a voxel
        '''

        voxel_map_size = np.asarray(np.ceil(np.array(voxel_map_size) / voxel_size), np.int32)
        center_x, center_y, center_z = voxel_map_center

        x_begin, x_end = [center_x + sign * 0.5 * voxel_map_size[0] * voxel_size for sign in [-1, 1]]
        y_begin, y_end = [center_y + sign * 0.5 * voxel_map_size[1] * voxel_size for sign in [-1, 1]]
        z_begin, z_end = [center_z + sign * 0.5 * voxel_map_size[2] * voxel_size for sign in [-1, 1]]

        voxel_map = np.zeros(shape=(*voxel_map_size, 7))
        for point in points:
            x, y, z, r, g, b = point

            if x_begin < x < x_end and y_begin < y < y_end and z_begin < z < z_end:
                voxel_map[math.floor((x - x_begin) / voxel_size), math.floor((y - y_begin) / voxel_size),
                          math.floor((z - z_begin) / voxel_size)] += [r, g, b, x, y, z, 1]

        voxel_map[:, :, :, :-1] \
            = voxel_map[:, :, :, :-1] / np.expand_dims(np.clip(voxel_map[:, :, :, -1], 1, None), axis=3)
        return voxel_map

    @staticmethod
    def read_kitti_3d_object(path, convert_format=True):
        '''
        Reads kitti 3d Object Labels
        :param path: Path of the label.txt file
        :param convert_format: If True, object format will be converted to the DepthVisualizer format which is on
        the following form
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        :return: A List of objects on either Kitti or DepthVisualizer format, based on convert_format argument
        '''
        objects = []
        with open(path, "r") as f:
            for line in f:
                object_label = line.split(" ")[0]
                if not (object_label == "DontCare"):
                    object_data = [x.rstrip() for x in line.split(" ")]
                    object_data[1:] = [float(x) for x in object_data[1:]]
                    objects.append(object_data)

        if convert_format:
            objects = Utils.convert_objects_from_kitti_format(objects)
        return objects


class DepthRenderer:
    def __init__(self, frame_width=1280, frame_height=720, window_name="Depth Visualizer",
                 camera_move_speed=10, camera_turn_speed=30, point_size=3, camera_fov=80, background_color=[60, 60, 60]):
        self.frame_width, self.frame_height = frame_width, frame_height
        self.window_name = window_name
        self.camera_move_speed, self.camera_turn_speed = camera_move_speed, camera_turn_speed
        self.__point_size = point_size
        self.__camera_fov = camera_fov
        self.__background_color = np.array(background_color, np.float32) / 256.0
        self.last_render_time = time.time()

        self.__init_glfw()
        self.__window = self.__init_window()
        self.__shader_program = self.__init_shaders()
        self.__vao_pointer = self.__init_opengl_vars()

        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glPointSize(self.__point_size)
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
                                          3, 2, 6,
                                          0, 4, 7
                                          ], np.int32)
        self.__voxel_indicies_data = vbo.VBO(self.__voxel_indicies, target=GL.GL_ELEMENT_ARRAY_BUFFER)
        self.__voxel_indicies_data.create_buffers()

        self.__camera_coords = glm.vec3(0, 0, 0)
        self.__camera_front_direction = glm.vec3(0, 0, 1)
        self.__camera_up = glm.vec3(0, 1, 0)

        self.camera_yaw = -90
        self.camera_pitch = 0

        self.__mouse_pos = [self.frame_width / 2, self.frame_height / 2]

        self.__update_shader_parameters()
        self.change_camera_rotation(0, 0)

        self.break_render_loop = False
        self.last_break_time = time.time()  # Used for input debounce delay

        self.render_points = True
        self.render_lines = True
        self.render_voxels = True

        self.is_window_hidden = False
        self.was_window_hidden = False

    def __init_glfw(self):
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    def hide_window(self):
        '''
        Hides renderer window
        :return: None
        '''
        self.is_window_hidden = True
        glfw.hide_window(self.__window)
        glfw.poll_events()

    def show_window(self):
        '''
        Shows renderer window
        :return: None
        '''
        self.is_window_hidden = False
        glfw.show_window(self.__window)

    def __init_window(self):
        window = glfw.create_window(self.frame_width, self.frame_height, self.window_name, None, None)
        glfw.make_context_current(window)
        GL.glViewport(0, 0, self.frame_width, self.frame_height)
        glfw.set_framebuffer_size_callback(window, self.__framebuffer_size_callback)
        return window

    def __framebuffer_size_callback(self, window, width, height):
        GL.glViewport(0, 0, width, height)

    def __mouse_callback(self, window, xpos, ypos):
        old_xpos, old_ypos = self.__mouse_pos
        x_offset = xpos - old_xpos
        y_offset = old_ypos - ypos
        self.__mouse_pos = [xpos, ypos]

        x_offset, y_offset = [x * self.camera_turn_speed for x in (x_offset, y_offset)]
        self.change_camera_rotation(x_offset, y_offset)

    def __init_shaders(self):
        vertex_shader = shaders.compileShader(vertex_shader_source, GL.GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(fragment_shader_source, GL.GL_FRAGMENT_SHADER)

        shader_program = shaders.compileProgram(vertex_shader, fragment_shader)
        GL.glUseProgram(shader_program)
        return shader_program

    def __init_opengl_vars(self):
        vao = GL.glGenVertexArrays(1)
        return vao

    def __update_shader_parameters(self):
        view_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "view")
        view_matrix = glm.lookAt(self.__camera_coords, self.__camera_coords + self.__camera_front_direction, self.__camera_up)
        GL.glUniformMatrix4fv(view_matrix_loc, 1, GL.GL_TRUE, np.ascontiguousarray(view_matrix, np.float32))

        perspective_matrix = glm.perspective(glm.radians(self.__camera_fov/2), self.frame_width / self.frame_height,
                                             0.1, 200)

        projection_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "projection")
        GL.glUniformMatrix4fv(projection_matrix_loc, 1, GL.GL_TRUE,
                              np.ascontiguousarray(perspective_matrix, np.float32))

    def __handle_vertex_attrib(self):
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(self.__point_data.data.itemsize * 3))
        GL.glEnableVertexAttribArray(1)

    def __render_points(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_POINT)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glUseProgram(self.__shader_program)

        self.__point_data.bind()
        self.__handle_vertex_attrib()
        GL.glDrawArrays(GL.GL_POINTS, 0, len(self.__point_data.data))

        GL.glBindVertexArray(0)

    def __render_lines(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
        GL.glBindVertexArray(self.__vao_pointer)
        self.__line_data.bind()

        self.__handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawArrays(GL.GL_LINES, 0, len(self.__line_data.data))
        GL.glBindVertexArray(0)

    def __render_voxels(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
        GL.glBindVertexArray(self.__vao_pointer)

        self.__voxel_indicies_data.bind()
        self.__voxel_data.bind()

        self.__handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawElements(GL.GL_TRIANGLES, len(self.__voxel_data.data), GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(0)

    def __convert_point_data_z_axis(self, point_data):
        converted_data = np.asarray(point_data, np.float32).reshape(-1, 6)
        converted_data[:, 2] *= -1
        converted_data = converted_data.flatten()
        return converted_data

    def set_points(self, points, filter_points_above=None):
        '''
        This is equivalent to calling reset_points and clear_points sequentially

        Sets given points as the point data of the renderer
        :param points: List of points where each point is of following format [x, y, z, r, g, b]
        :param filter_points_above: If not none, filters points whose Y value is above specified value
        :return: None
        '''
        points = np.array(points, np.float32)
        self.clear_points()
        self.add_points(points, filter_points_above=filter_points_above)

    def add_points(self, points, filter_points_above=None):
        '''
        Adds points to renderers point data
        :param points: List of points where each point is of following format [x, y, z, r, g, b]
        :param filter_points_above: If not none, filters points whose Y value is above specified value
        :return: None
        '''
        points = np.array(points, np.float32)

        if filter_points_above is not None:
            points = points[points[:, 1] < filter_points_above]

        old_points = self.__point_data.data

        points = self.__convert_point_data_z_axis(points)
        if len(old_points) == 0:
            new_points = points
        else:
            new_points = np.append(old_points, points, axis=0)
        print(new_points)
        self.__point_data.set_array(new_points)

    def clear_points(self):
        '''
        Clears renderers point data
        :return: None
        '''
        self.__point_data.set_array(np.array([], np.float32))

    def set_lines(self, lines):
        '''
        This is equivalent calling clear_lines and add_lines sequentially

        Sets given lines as the line data of the renderer
        :param lines: List of line data where each line data is of following format
        [x_begin, y_begin, z_begin, r, g, b], [x_end, y_end, z_end, r, g, b]
        :return: None
        '''
        self.clear_lines()
        self.add_lines(lines)

    def add_lines(self, lines):
        '''
        Add's line data to renderers line data
        :param lines: List of line data where each line data is of following format
        [x_begin, y_begin, z_begin, r, g, b], [x_end, y_end, z_end, r, g, b]
        :return: None
        '''
        old_lines = self.__line_data.data

        lines = np.array(lines)
        if len(lines.shape) > 1:
            lines = lines.flatten()

        lines = self.__convert_point_data_z_axis(lines)
        if len(old_lines) == 0:
            new_lines = lines
        else:
            new_lines = np.append(old_lines, lines, axis=0)
        self.__line_data.set_array(new_lines)

    def add_axis_lines(self, length=10, x_axis_color=[256, 0, 0], y_axis_color=[0, 256, 0], z_axis_color=[0, 0, 256]):
        '''
        Adds lines for representing axis directions to center of the 3D world, lines will start to turn white in
        increasing direction
        :param length: Length of axis lines
        :param x_axis_color: Color of x axis [r, g, b]
        :param y_axis_color: Color of y axis [r, g, b]
        :param z_axis_color: Color of z axis [r, g, b]
        :return: None
        '''

        length /= 2
        x_axis_color = np.array(x_axis_color) / 256.0
        y_axis_color = np.array(y_axis_color) / 256.0
        z_axis_color = np.array(z_axis_color) / 256.0

        lines = [[-length, 0, 0, *x_axis_color], [length, 0, 0, 1, 1, 1], [0, -length, 0, *y_axis_color],
                 [0, length, 0, 1, 1, 1],
                 [0, 0, -length, *z_axis_color], [0, 0, length, 1, 1, 1]]
        lines = np.array(lines, np.float32).flatten()
        self.add_lines(lines)

    def clear_lines(self):
        '''
        Clears renderers line data
        :return: None
        '''
        self.__line_data.set_array(np.array([], np.float32))
        self.__voxel_indicies_data.set_array(np.array([], np.int32))

    def set_voxels(self, voxels):
        '''
        This is equivalent calling clear_voxels and add_voxels sequentially

        Sets given voxel as the voxel data of the renderer
        :param voxels: List of voxels where each voxel is of form [x, y, z, size, r, g b]
        :return: None
        '''
        self.clear_voxels()
        self.add_voxels(voxels)

    def __convert_voxels_to_opengl_format(self, voxels):
        voxels = np.array(voxels, np.float32)
        voxels = voxels.reshape(-1, 7)
        formatted_voxels = np.zeros(shape=(len(voxels), 6 * 8))

        for i in range(8):
            formatted_voxels[:, i * 6 + 3:i * 6 + 6] = voxels[:, 4:7]

        size = voxels[:, 3] / 2
        formatted_voxels[:, 0] = voxels[:, 0] - size  # x0
        formatted_voxels[:, 1] = voxels[:, 1] + size  # y0
        formatted_voxels[:, 2] = voxels[:, 2] - size  # z0

        formatted_voxels[:, 0 + 6] = voxels[:, 0] + size  # x1
        formatted_voxels[:, 1 + 6] = voxels[:, 1] + size  # y1
        formatted_voxels[:, 2 + 6] = voxels[:, 2] - size  # z1

        formatted_voxels[:, 0 + 6 * 2] = voxels[:, 0] + size  # x2
        formatted_voxels[:, 1 + 6 * 2] = voxels[:, 1] - size  # y2
        formatted_voxels[:, 2 + 6 * 2] = voxels[:, 2] - size  # z2

        formatted_voxels[:, 0 + 6 * 3] = voxels[:, 0] - size  # x3
        formatted_voxels[:, 1 + 6 * 3] = voxels[:, 1] - size  # y3
        formatted_voxels[:, 2 + 6 * 3] = voxels[:, 2] - size  # z3

        formatted_voxels[:, 0 + 6 * 4] = voxels[:, 0] - size  # x4
        formatted_voxels[:, 1 + 6 * 4] = voxels[:, 1] + size  # y4
        formatted_voxels[:, 2 + 6 * 4] = voxels[:, 2] + size  # z4

        formatted_voxels[:, 0 + 6 * 5] = voxels[:, 0] + size  # x5
        formatted_voxels[:, 1 + 6 * 5] = voxels[:, 1] + size  # y5
        formatted_voxels[:, 2 + 6 * 5] = voxels[:, 2] + size  # z5

        formatted_voxels[:, 0 + 6 * 6] = voxels[:, 0] + size  # x6
        formatted_voxels[:, 1 + 6 * 6] = voxels[:, 1] - size  # y6
        formatted_voxels[:, 2 + 6 * 6] = voxels[:, 2] + size  # z6

        formatted_voxels[:, 0 + 6 * 7] = voxels[:, 0] - size  # x7
        formatted_voxels[:, 1 + 6 * 7] = voxels[:, 1] - size  # y7
        formatted_voxels[:, 2 + 6 * 7] = voxels[:, 2] + size  # z7
        return formatted_voxels.flatten()

    def add_voxels(self, voxels):
        '''
        Adds voxels to renderers voxel data
        :param voxels: List of voxels where each voxel is of form [x, y, z, size, r, g b]
        :return: None
        '''
        voxels = self.__convert_voxels_to_opengl_format(voxels)
        old_voxels = self.__voxel_data.data

        voxels = self.__convert_point_data_z_axis(voxels)
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

    def clear_voxels(self):
        '''
        Clears renderers voxel data
        :return: None
        '''
        self.__voxel_data.set_array(np.array([], np.float32))

    def clear_all(self):
        '''
        Clears renderers voxel, line and point data
        :return: None
        '''
        self.clear_voxels()
        self.clear_lines()
        self.clear_points()

    @staticmethod
    def __convert_2d_bbox_to_depthrenderer_format(bbox):
        '''
        Takes a 2D bounding box of format [left, top, right, bottom] and converts it to DepthRenderer format of
        [center_x, center_y, half_width, half_height]
        :param bbox: 4 element list of format [left, top, right, bottom]
        :return: bbox in DepthRenderer format [center_x, center_y, half_width, half_height]
        '''

        half_width = (bbox[2] - bbox[0]) / 2
        half_height = (bbox[3] - bbox[1]) / 2

        center_x = (bbox[2] + bbox[0]) / 2
        center_y = (bbox[3] + bbox[1]) / 2

        return [center_x, center_y, half_width, half_height]

    def add_2d_bbox(self, bbox, principal_point, focal_length_in_pixels=715, color=[255, 255, 255], depth=10,
                    draw_3d=False, max_depth=80):
        '''
        Draws a 2d bbox on 3d space
        :param bbox: [left, top, right, bottom]
        :param principal_point: Center of the image
        :param focal_length_in_pixels: Camera's focal length measured in pixels
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :param depth: Depth of first bbox
        :param draw_3d: Draws a 2nd bbox whose location is based on max_depth argument
        :param max_depth: Depth of 2nd bbox, only effective when draw_3d is set to True
        :return: None
        '''
        color = np.array(color) / 256.0
        box_data = DepthRenderer.__convert_2d_bbox_to_depthrenderer_format(bbox)

        quad_data = [[box_data[0] - box_data[2], box_data[1] + box_data[3]],
                     [box_data[0] + box_data[2], box_data[1] + box_data[3]],
                     [box_data[0] + box_data[2], box_data[1] - box_data[3]],
                     [box_data[0] - box_data[2], box_data[1] - box_data[3]]]
        first_quad_points = np.array([Utils.convert_depth_pixel_to_point(x, y, depth, focal_length_in_pixels,
                                                                         principal_point, color)
                                      for x, y in quad_data])

        self.add_lines(np.array([first_quad_points[0], first_quad_points[1], first_quad_points[1], first_quad_points[2],
                                 first_quad_points[2], first_quad_points[3], first_quad_points[3],
                                 first_quad_points[0]]).flatten())

        if draw_3d:
            second_quad_points = np.array([Utils.convert_depth_pixel_to_point(x, y, max_depth, focal_length_in_pixels,
                                                                              principal_point, color)
                                           for x, y in quad_data])
            self.add_lines(
                np.array([second_quad_points[0], second_quad_points[1], second_quad_points[1], second_quad_points[2],
                          second_quad_points[2], second_quad_points[3], second_quad_points[3],
                          second_quad_points[0]]).flatten())

            self.add_lines(np.array([[first_quad_points[x], second_quad_points[x]] for x in range(4)]).flatten())

    def add_3d_object_bbox(self, bbox_data, color=[255, 255, 255]):
        '''
        Draws 3d bounding box with direction indicator
        :param bbox_data: [center_x, center_y, center_z, width, height, length, y_rotation]
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :return: None
        '''
        color = np.array(color) / 256.0
        first_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2],
                               [bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2]]

        second_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2]]

        matrix = glm.translate(glm.mat4(1), glm.vec3(*bbox_data[:3]))
        matrix = glm.rotate(matrix, bbox_data[-1], glm.vec3(0, 1, 0))

        first_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in first_rect_centered]
        second_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in second_rect_centered]

        front_point = list(matrix * glm.vec4(0, 0, bbox_data[5] * 0.8, 1))[:3]

        lines = [first_rect_centered[0], first_rect_centered[1],
                 first_rect_centered[1], first_rect_centered[2],
                 first_rect_centered[2], first_rect_centered[3],
                 first_rect_centered[3], first_rect_centered[0],
                 second_rect_centered[0], second_rect_centered[1],
                 second_rect_centered[1], second_rect_centered[2],
                 second_rect_centered[2], second_rect_centered[3],
                 second_rect_centered[3], second_rect_centered[0],
                 first_rect_centered[0], front_point,
                 first_rect_centered[1], front_point,
                 first_rect_centered[2], front_point,
                 first_rect_centered[3], front_point
                 ]
        lines = np.append(lines, np.array([[first_rect_centered[x], second_rect_centered[x]] for x
                                           in range(4)]).reshape(-1, 3), axis=0)
        lines = np.array([[x, y, z, color[0], color[1], color[2]] for x, y, z in lines])
        self.add_lines(lines)

    def add_3d_bbox(self, bbox_data, color=[255, 255, 255]):
        '''
        Draws 3d bounding box
        :param bbox_data: [center_x, center_y, center_z, width, height, length, y_rotation]
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :return: None
        '''
        color = np.array(color) / 256.0
        first_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2],
                               [bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2]]

        second_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2]]

        matrix = glm.translate(glm.mat4(1), glm.vec3(*bbox_data[:3]))
        matrix = glm.rotate(matrix, bbox_data[-1], glm.vec3(0, 1, 0))

        first_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in first_rect_centered]
        second_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in second_rect_centered]

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
        '''
        Sets the camera location at given coordinates
        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :return: None
        '''
        self.__camera_coords = glm.vec3(x, y, -z)

    def move_forward(self, meters):
        '''
        Moves the camera forward towards the look direction by specified meters
        :param meters: Amount of meters to move
        :return:
        '''
        print(self.__camera_front_direction)
        self.__camera_coords += self.__camera_front_direction * meters

    def move_right(self, meters):
        '''
        Moves the camera to the right of the look direction by specified meters
        :param meters: Amount of meters to move
        :return:
        '''
        self.__camera_coords += glm.normalize(glm.cross(self.__camera_front_direction, self.__camera_up)) * meters

    def look_up(self, angles):
        '''
        Makes the camera turn upwards by specified angle
        :param angles: Specifies how much to rotate camera
        :return:
        '''
        self.change_camera_rotation(0, angles)

    def look_right(self, angles):
        '''
        Makes the camera turn right by specified angle
        :param angles: Specifies how much to rotate camera
        :return:
        '''
        self.change_camera_rotation(angles, 0)

    def __process_input(self, frame_delay=0.016, process_movement=True):
        if glfw.get_key(self.__window, glfw.KEY_ESCAPE) == glfw.PRESS:
            current_time = time.time()
            if current_time - self.last_break_time > 0.5:
                self.break_render_loop = True
                self.last_break_time = current_time
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

        if glfw.get_key(self.__window, glfw.KEY_1) == glfw.PRESS:
            self.render_points = True
        if glfw.get_key(self.__window, glfw.KEY_2) == glfw.PRESS:
            self.render_points = False
        if glfw.get_key(self.__window, glfw.KEY_3) == glfw.PRESS:
            self.render_lines = True
        if glfw.get_key(self.__window, glfw.KEY_4) == glfw.PRESS:
            self.render_lines = False
        if glfw.get_key(self.__window, glfw.KEY_5) == glfw.PRESS:
            self.render_voxels = True
        if glfw.get_key(self.__window, glfw.KEY_6) == glfw.PRESS:
            self.render_voxels = False

    def add_voxel_map(self, voxel_map, voxel_map_center, voxel_size, filter_empty_voxels=True):
        '''
        Adds given voxel map to renderers voxel data
        :param voxel_map: A 3D array in x-y-z format where each element is list of size 3 that represents color
        :param voxel_map_center: Center of the voxel map
        :param voxel_size: Size of voxels in meters
        :param filter_empty_voxels: Filters out empty voxels if enabled
        :return: None
        '''
        x, y, z = np.meshgrid(np.arange(0, voxel_map.shape[0]), np.arange(0, voxel_map.shape[1]),
                              np.arange(0, voxel_map.shape[2]), indexing='ij')
        xyz_coordinates = np.array([x.flatten(), y.flatten(), z.flatten()], np.float32).T
        xyz_coordinates[:, 0] -= voxel_map.shape[0] / 2
        xyz_coordinates[:, 1] -= voxel_map.shape[1] / 2
        xyz_coordinates[:, 2] -= voxel_map.shape[2] / 2

        xyz_coordinates *= voxel_size
        xyz_coordinates += voxel_size / 2

        xyz_coordinates += voxel_map_center
        voxel_data = np.zeros(shape=(voxel_map.shape[0] * voxel_map.shape[1] * voxel_map.shape[2], 7))
        flattened_map = voxel_map.reshape(-1, 7)
        voxel_data[:, :3] = xyz_coordinates
        voxel_data[:, 3] = voxel_size
        voxel_data[:, 4:] = flattened_map[:, :3]

        if filter_empty_voxels:
            voxel_data = voxel_data[voxel_data[:, -1] != 0]

        self.add_voxels(voxel_data.flatten())

    def render(self, enable_controls=False):
        '''
        Renders existing points, lines and voxels to frame
        :param enable_controls: If true, camera will be allowed to move using keyboard
        :return: None
        '''
        current_time = time.time()
        self.__process_input(current_time - self.last_render_time, enable_controls)
        self.last_render_time = current_time

        GL.glClearColor(*self.__background_color, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.__update_shader_parameters()
        if self.render_points:
            self.__render_points()
        if self.render_lines:
            self.__render_lines()
        if self.render_voxels:
            self.__render_voxels()

        # print(self.__camera_coords)
        glfw.swap_buffers(self.__window)
        glfw.poll_events()

    def should_continue_rendering(self):
        '''
        A function for custom loops, custom loops should stop when this method returns false
        :return: boolean
        '''
        return not glfw.window_should_close(self.__window)

    def get_rendered_frame(self):
        '''
        Extracts image from renderers window
        :return: A numpy array that contains RGB frame on shape [frame_height, frame_width, 3]
        '''
        GL.glReadBuffer(GL.GL_FRONT)
        data = GL.glReadPixels(0, 0, self.frame_width, self.frame_height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        image = np.frombuffer(data, dtype=np.uint8).reshape(self.frame_height, self.frame_width, 3)
        image = np.flip(image, 0)
        return image

    def render_loop(self, enable_controls=True):
        '''
        A simple render loop, mainly used for controlling camera, loop will exit when user pressed ESC if controls are
        enabled
        :param enable_controls: If true, camera will be allowed to move using keyboard
        :return: None
        '''
        if self.is_window_hidden:
            self.was_window_hidden = True
            self.show_window()
        while self.should_continue_rendering() and not self.break_render_loop:
            self.render(enable_controls)
        if self.break_render_loop:
            self.break_render_loop = False

        if self.was_window_hidden:
            self.hide_window()
            self.was_window_hidden = False

    def close(self):
        '''
        Used for closing renderer
        :return: None
        '''
        glfw.terminate()

    def change_camera_rotation(self, yaw_offset, pitch_offset):
        '''
        Turns the camera by specified yaw and pitch values
        :param yaw_offset: Yaw value in degrees
        :param pitch_offset: Pitch value in degrees
        :return: None
        '''
        self.camera_yaw += yaw_offset
        self.camera_pitch += pitch_offset
        self.camera_pitch = np.clip(self.camera_pitch, -89, 89)
        direction = glm.vec3()
        direction.x = math.cos(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        direction.y = math.sin(glm.radians(self.camera_pitch))
        direction.z = math.sin(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        print(direction)
        self.__camera_front_direction = glm.normalize(direction)
=======
import glfw
import OpenGL.GL as GL
from OpenGL.GL import shaders
from OpenGL.arrays import vbo
import math
import glm
import ctypes
import time
import numpy as np
from PIL import Image

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


class Utils:
    def __init__(self):
        pass

    @staticmethod
    def convert_depth_pixel_to_point(x, y, depth, focal_length_in_pixels, principal_point
                                     , rgb=[255, 255, 255], is_depth_along_z=True):
        '''
        This method is used for converting a single pixel to 3D point
        :param x: X coordinate of pixel in image plane
        :param y: Y coordinate of the pixel in image plane
        :param depth: Depth of the pixel
        :param focal_length_in_pixels: Focal length of the camera measured in pixels
        :param principal_point: Center of the image
        :param rgb: Color of the pixel
        :param is_depth_along_z: If True, z coordinate of the pixel will be equal to depth, otherwise depth will be
        calculated as distance between camera and pixel
        :return: A point of form [x, y, z, r, g, b]
        '''
        rgb = list(rgb)

        x_dist = x - principal_point[1]
        y_dist = y - principal_point[0]

        z = depth
        if not is_depth_along_z:
            pixel_dist = (x_dist ** 2 + y_dist ** 2) ** 0.5
            focal_target = (focal_length_in_pixels ** 2 + pixel_dist ** 2) ** 0.5
            z = depth * focal_length_in_pixels / focal_target

        x = x_dist / focal_length_in_pixels * z
        y = y_dist / focal_length_in_pixels * z

        return [x, -y, z] + rgb

    @staticmethod
    def convert_point_to_pixel(point, focal_length_in_pixels, principal_point):
        '''
        This method is used for converting a point in [x, y, z, r, g, b] to a RGBD pixel
        :param point: A point of form [x, y, z, r, g, b]
        :param focal_length_in_pixels: Focal length of the camera measured in pixels
        :param principal_point: Center of the image
        :return: An RGBD pixel in shape [y, x, depth, r, g, b]
        '''

        x_dist = point[0] / point[2] * focal_length_in_pixels
        y_dist = point[1] / point[2] * focal_length_in_pixels

        y_coord = principal_point[0] - y_dist
        x_coord = principal_point[1] + x_dist

        return [y_coord, x_coord] + point[2:]

    @staticmethod
    def read_kitti_calibration(path):
        '''
        This method reads Kitti calibration file and converts it to a dictionary
        :param path: Path for the kitti dataset calibration txt
        :return: Dictionary containing calibration information
        '''
        calib_data = {}
        with open(path, 'r') as f:
            for line in f.readlines():
                if ':' in line:
                    key, value = line.split(":", 1)
                    calib_data[key] = np.array([float(x) for x in value.split()])

        R0 = calib_data['P0'].reshape(3, 4)
        focal_length = R0[1, 1]
        principal_point = R0[1, 2], R0[0, 2]

        calib_data["focal_length"] = focal_length
        calib_data["principal_point"] = principal_point

        return calib_data

    @staticmethod
    def convert_depthmap_to_points(depth_map, focal_length_in_pixels=None, principal_point=None, rgb_image=None,
                                   is_depth_along_z=True):
        '''
        Converts given depth map to point cloud
        :param depth_map: A grayscale image where values are equal to depth in meters
        :param focal_length_in_pixels: Focal lenght of the camera measured in pixels
        :param principal_point: Center of image
        :param rgb_image: Colored image that matches depth map, used for coloring points
        :param is_depth_along_z: If True, z coordinate of the pixel will be equal to depth, otherwise depth will be
        calculated as distance between camera and pixel
        :return: List of points where each point is of form [x, y, z, r, g, b]
        '''
        if focal_length_in_pixels is None:
            focal_length_in_pixels = 715
        if principal_point is None:
            principal_point = [depth_map.shape[0] / 2, depth_map.shape[1] / 2]

        if depth_map.shape[0] == 1:
            depth_map = np.swapaxes(depth_map, 0, 1).swapaxes(1, 2)

        points = np.ones(shape=(depth_map.shape[0] * depth_map.shape[1], 6))  # A point contains x,y,z,r,g,b
        if rgb_image is None:
            points[:, 3:6] = [0.5, 0.7, 1]
        else:
            if rgb_image.shape[0] == 3:
                rgb_image = np.swapaxes(rgb_image, 0, 1).swapaxes(1, 2)
            rgb_image = rgb_image.reshape(-1, 3)
            points[:, 3:6] = rgb_image / 256.0

        y, x = np.meshgrid(np.arange(0, depth_map.shape[1]), np.arange(0, depth_map.shape[0]))
        yx_coordinates = np.array([x.flatten(), y.flatten()], np.float32).T
        yx_coordinates += -1 * np.array(principal_point)
        yx_coordinates = np.flip(yx_coordinates, 1)

        points[:, 0:2] = yx_coordinates
        points[:, 2] = depth_map.flatten()

        pixel_dist = (points[:, 0] ** 2 + points[:, 1] ** 2) ** 0.5
        focal_target_dist = (focal_length_in_pixels ** 2 + pixel_dist ** 2) ** 0.5

        if not is_depth_along_z:
            points[:, 2] = points[:, 2] * focal_length_in_pixels / focal_target_dist

        points[:, 0] = points[:, 0] * points[:, 2] / focal_length_in_pixels
        points[:, 1] = points[:, 1] * points[:, 2] / focal_length_in_pixels

        points[:, 1] *= -1
        return points

    @staticmethod
    def read_kitti_point_cloud(path, calib, color=[255, 255, 255]):
        '''
        Reads kitti point cloud file and converts its coordinate space from velodyne to camera format
        :param path: Path of the point cloud file
        :param calib: Calibration dictionary, can be read from file using read_kitti_calibration method
        :param color: Color of points
        :return: Returns a list of points where each point is of format [x, y, z, r, g, b]
        '''
        color = np.array(color) / 256.0
        points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
        points = points[:, :3]
        new_points = np.ones(shape=(len(points), 4))
        new_points[:, :3] = points

        Tr_velo_to_cam = np.zeros((4, 4))
        Tr_velo_to_cam[3, 3] = 1
        Tr_velo_to_cam[:3, :4] = calib['Tr_velo_to_cam'].reshape(3, 4)

        converted_points = Tr_velo_to_cam.dot(new_points.T).T

        point_cloud = np.zeros((points.shape[0], 6))
        point_cloud[:, :3] = converted_points[:, :3]
        point_cloud[:, 1] *= -1

        point_cloud[:, 3:] = color
        return point_cloud

    @staticmethod
    def read_depth_map(path):
        depth_map = np.asarray(Image.open(path), np.float32)
        depth_map = np.expand_dims(depth_map, axis=2) / 256.0
        return depth_map

    @staticmethod
    def convert_objects_from_kitti_format(objects):
        '''
        Takes a list of objects that is on the kitti format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D Width, 3D Height,
        3D Length, 3D X, 3D Y, 3D Z, 3D Rotation]
        and converts them to DepthVisualizer format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        :param objects: List of objects where each object is on the kitti format
        :return: List of objects where each object is on the DepthVisualizer format
        '''
        converted_objects = []
        for object in objects:
            object = object.copy()
            object_data_array = np.array([float(x) for x in object[8:]], np.float32)
            object[8 + 0:8 + 3] = object_data_array[0 + 3:3 + 3]
            object[8 + 1] *= -1

            object[11:14] = object_data_array[0:3]

            object[11], object[12] = object[12], object[11]

            object[8 + 1] += object[12] / 2

            object[14] += math.radians(90)

            converted_objects.append(object)
        return converted_objects

    @staticmethod
    def convert_objects_to_kitti_format(objects):
        '''
        Takes a list of objects that is on the DepthVisualizer format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        and converts them to Kitti format,
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D Width, 3D Height,
        3D Length, 3D X, 3D Y, 3D Z, 3D Rotation]
        :param objects: List of objects where each object is on the DepthVisualizer format
        :return: List of objects where each object is on the Kitti format
        '''
        converted_objects = []

        for object in objects.copy():
            object_clone = object.copy()

            object_clone[8 + 1] -= object[12] / 2
            object_clone[8 + 1] *= -1

            object[11], object[12] = object[12], object[11]

            object_clone[11:14] = object_clone[8:11]
            object_clone[8:11] = object[11:14]

            object_clone[14] -= math.radians(90)

            converted_objects.append(object_clone)

        return converted_objects

    @staticmethod
    def convert_points_to_voxel_map(points, voxel_map_center, voxel_map_size, voxel_size):
        '''
        Converts points that are inside specified 3D box into voxels
        :param points: List of points where each point has following format [x, y, z, r, g, b]
        :param voxel_map_center: Center coordinates of the area that will be voxellized in format [x, y, z]
        :param voxel_map_size: Size of the area that will be voxellized in format [width, height, length]
        :param voxel_size: Size of the each voxel
        :return: A 3D array in x-y-z format where each element is list of size 7 where each element is of following
        format [r , g, b, mean x, mean y, mean z, point_count]. point_count is equal to number of points that are
        inside a voxel
        '''

        voxel_map_size = np.asarray(np.ceil(np.array(voxel_map_size) / voxel_size), np.int32)
        center_x, center_y, center_z = voxel_map_center

        x_begin, x_end = [center_x + sign * 0.5 * voxel_map_size[0] * voxel_size for sign in [-1, 1]]
        y_begin, y_end = [center_y + sign * 0.5 * voxel_map_size[1] * voxel_size for sign in [-1, 1]]
        z_begin, z_end = [center_z + sign * 0.5 * voxel_map_size[2] * voxel_size for sign in [-1, 1]]

        voxel_map = np.zeros(shape=(*voxel_map_size, 7))
        for point in points:
            x, y, z, r, g, b = point

            if x_begin < x < x_end and y_begin < y < y_end and z_begin < z < z_end:
                voxel_map[math.floor((x - x_begin) / voxel_size), math.floor((y - y_begin) / voxel_size),
                          math.floor((z - z_begin) / voxel_size)] += [r, g, b, x, y, z, 1]

        voxel_map[:, :, :, :-1] \
            = voxel_map[:, :, :, :-1] / np.expand_dims(np.clip(voxel_map[:, :, :, -1], 1, None), axis=3)
        return voxel_map

    @staticmethod
    def read_kitti_3d_object(path, convert_format=True):
        '''
        Reads kitti 3d Object Labels
        :param path: Path of the label.txt file
        :param convert_format: If True, object format will be converted to the DepthVisualizer format which is on
        the following form
        [Type, Truncation, Occlusion, Observing  Angle, 2D Left, 2D Top, 2D Right, 2D Bottom, 3D X, 3D Y, 3D Z,
        3D Width, 3D Height, 3D Length, 3D Rotation]
        :return: A List of objects on either Kitti or DepthVisualizer format, based on convert_format argument
        '''
        objects = []
        with open(path, "r") as f:
            for line in f:
                object_label = line.split(" ")[0]
                if not (object_label == "DontCare"):
                    object_data = [x.rstrip() for x in line.split(" ")]
                    object_data[1:] = [float(x) for x in object_data[1:]]
                    objects.append(object_data)

        if convert_format:
            objects = Utils.convert_objects_from_kitti_format(objects)
        return objects


class DepthRenderer:
    def __init__(self, frame_width, frame_height, window_name="Depth Visualizer",
                 camera_move_speed=20, camera_turn_speed=90, point_size=3, camera_fov=130):
        self.frame_width, self.frame_height = frame_width, frame_height
        self.window_name = window_name
        self.camera_move_speed, self.camera_turn_speed = camera_move_speed, camera_turn_speed
        self.__point_size = point_size
        self.__camera_fov = camera_fov
        self.last_render_time = time.time()

        self.__init_glfw()
        self.__window = self.__init_window()
        self.__shader_program = self.__init_shaders()
        self.__vao_pointer = self.__init_opengl_vars()

        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glPointSize(self.__point_size)
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
                                          3, 2, 6,
                                          0, 4, 7
                                          ], np.int32)
        self.__voxel_indicies_data = vbo.VBO(self.__voxel_indicies, target=GL.GL_ELEMENT_ARRAY_BUFFER)
        self.__voxel_indicies_data.create_buffers()

        self.__camera_coords = glm.vec3(0, 10, 10)
        self.__camera_front = glm.vec3(0, 0, -1)
        self.__camera_up = glm.vec3(0, 1, 0)

        self.camera_yaw = -90
        self.camera_pitch = -10

        self.__mouse_pos = [self.frame_width / 2, self.frame_height / 2]

        self.__update_shader_parameters()
        self.change_camera_rotation(0, 0)

        self.break_render_loop = False
        self.last_break_time = time.time()  # Used for input debounce delay

        self.render_points = True
        self.render_lines = True
        self.render_voxels = True

        self.is_window_hidden = False
        self.was_window_hidden = False

    def __init_glfw(self):
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    def hide_window(self):
        '''
        Hides renderer window
        :return: None
        '''
        self.is_window_hidden = True
        glfw.hide_window(self.__window)
        glfw.poll_events()

    def show_window(self):
        '''
        Shows renderer window
        :return: None
        '''
        self.is_window_hidden = False
        glfw.show_window(self.__window)

    def __init_window(self):
        window = glfw.create_window(self.frame_width, self.frame_height, self.window_name, None, None)
        glfw.make_context_current(window)
        GL.glViewport(0, 0, self.frame_width, self.frame_height)
        glfw.set_framebuffer_size_callback(window, self.__framebuffer_size_callback)
        return window

    def __framebuffer_size_callback(self, window, width, height):
        GL.glViewport(0, 0, width, height)

    def __mouse_callback(self, window, xpos, ypos):
        old_xpos, old_ypos = self.__mouse_pos
        x_offset = xpos - old_xpos
        y_offset = old_ypos - ypos
        self.__mouse_pos = [xpos, ypos]

        x_offset, y_offset = [x * self.camera_turn_speed for x in (x_offset, y_offset)]
        self.change_camera_rotation(x_offset, y_offset)

    def __init_shaders(self):
        vertex_shader = shaders.compileShader(vertex_shader_source, GL.GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(fragment_shader_source, GL.GL_FRAGMENT_SHADER)

        shader_program = shaders.compileProgram(vertex_shader, fragment_shader)
        GL.glUseProgram(shader_program)
        return shader_program

    def __init_opengl_vars(self):
        vao = GL.glGenVertexArrays(1)
        return vao

    def __update_shader_parameters(self):
        view_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "view")
        view_matrix = glm.lookAt(self.__camera_coords, self.__camera_coords + self.__camera_front, self.__camera_up)
        GL.glUniformMatrix4fv(view_matrix_loc, 1, GL.GL_FALSE, np.ascontiguousarray(view_matrix, np.float32))

        perspective_matrix = glm.perspective(glm.radians(self.__camera_fov/2), self.frame_width / self.frame_height,
                                             0.1, 200)

        projection_matrix_loc = GL.glGetUniformLocation(self.__shader_program, "projection")
        GL.glUniformMatrix4fv(projection_matrix_loc, 1, GL.GL_FALSE,
                              np.ascontiguousarray(perspective_matrix, np.float32))

    def __handle_vertex_attrib(self):
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, self.__point_data.data.itemsize * 6,
                                 ctypes.c_void_p(self.__point_data.data.itemsize * 3))
        GL.glEnableVertexAttribArray(1)

    def __render_points(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_POINT)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glUseProgram(self.__shader_program)

        self.__point_data.bind()
        self.__handle_vertex_attrib()
        GL.glDrawArrays(GL.GL_POINTS, 0, len(self.__point_data.data))

        GL.glBindVertexArray(0)

    def __render_lines(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
        GL.glBindVertexArray(self.__vao_pointer)
        self.__line_data.bind()

        self.__handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawArrays(GL.GL_LINES, 0, len(self.__line_data.data))
        GL.glBindVertexArray(0)

    def __render_voxels(self):
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
        GL.glBindVertexArray(self.__vao_pointer)

        self.__voxel_indicies_data.bind()
        self.__voxel_data.bind()

        self.__handle_vertex_attrib()

        GL.glUseProgram(self.__shader_program)
        GL.glBindVertexArray(self.__vao_pointer)
        GL.glDrawElements(GL.GL_TRIANGLES, len(self.__voxel_data.data), GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(0)

    def __convert_point_data_z_axis(self, point_data):
        converted_data = np.asarray(point_data, np.float32).reshape(-1, 6)
        converted_data[:, 2] *= -1
        converted_data = converted_data.flatten()
        return converted_data

    def set_points(self, points, filter_points_above=None):
        '''
        This is equivalent to calling reset_points and clear_points sequentially

        Sets given points as the point data of the renderer
        :param points: List of points where each point is of following format [x, y, z, r, g, b]
        :param filter_points_above: If not none, filters points whose Y value is above specified value
        :return: None
        '''
        points = np.array(points, np.float32)
        self.clear_points()
        self.add_points(points, filter_points_above=filter_points_above)

    def add_points(self, points, filter_points_above=None):
        '''
        Adds points to renderers point data
        :param points: List of points where each point is of following format [x, y, z, r, g, b]
        :param filter_points_above: If not none, filters points whose Y value is above specified value
        :return: None
        '''
        points = np.array(points, np.float32)

        if filter_points_above is not None:
            points = points[points[:, 1] < filter_points_above]

        old_points = self.__point_data.data

        points = self.__convert_point_data_z_axis(points)
        if len(old_points) == 0:
            new_points = points
        else:
            new_points = np.append(old_points, points, axis=0)
        self.__point_data.set_array(new_points)

    def clear_points(self):
        '''
        Clears renderers point data
        :return: None
        '''
        self.__point_data.set_array(np.array([], np.float32))

    def set_lines(self, lines):
        '''
        This is equivalent calling clear_lines and add_lines sequentially

        Sets given lines as the line data of the renderer
        :param lines: List of line data where each line data is of following format
        [x_begin, y_begin, z_begin, r, g, b], [x_end, y_end, z_end, r, g, b]
        :return: None
        '''
        self.clear_lines()
        self.add_lines(lines)

    def add_lines(self, lines):
        '''
        Add's line data to renderers line data
        :param lines: List of line data where each line data is of following format
        [x_begin, y_begin, z_begin, r, g, b], [x_end, y_end, z_end, r, g, b]
        :return: None
        '''
        old_lines = self.__line_data.data

        lines = np.array(lines)
        if len(lines.shape) > 1:
            lines = lines.flatten()

        lines = self.__convert_point_data_z_axis(lines)
        if len(old_lines) == 0:
            new_lines = lines
        else:
            new_lines = np.append(old_lines, lines, axis=0)
        self.__line_data.set_array(new_lines)

    def add_axis_lines(self, length=10, x_axis_color=[256, 0, 0], y_axis_color=[0, 256, 0], z_axis_color=[0, 0, 256]):
        '''
        Adds lines for representing axis directions to center of the 3D world, lines will start to turn white in
        increasing direction
        :param length: Length of axis lines
        :param x_axis_color: Color of x axis [r, g, b]
        :param y_axis_color: Color of y axis [r, g, b]
        :param z_axis_color: Color of z axis [r, g, b]
        :return: None
        '''

        length /= 2
        x_axis_color = np.array(x_axis_color) / 256.0
        y_axis_color = np.array(y_axis_color) / 256.0
        z_axis_color = np.array(z_axis_color) / 256.0

        lines = [[-length, 0, 0, *x_axis_color], [length, 0, 0, 1, 1, 1], [0, -length, 0, *y_axis_color],
                 [0, length, 0, 1, 1, 1],
                 [0, 0, -length, *z_axis_color], [0, 0, length, 1, 1, 1]]
        lines = np.array(lines, np.float32).flatten()
        self.add_lines(lines)

    def clear_lines(self):
        '''
        Clears renderers line data
        :return: None
        '''
        self.__line_data.set_array(np.array([], np.float32))
        self.__voxel_indicies_data.set_array(np.array([], np.int32))

    def set_voxels(self, voxels):
        '''
        This is equivalent calling clear_voxels and add_voxels sequentially

        Sets given voxel as the voxel data of the renderer
        :param voxels: List of voxels where each voxel is of form [x, y, z, size, r, g b]
        :return: None
        '''
        self.clear_voxels()
        self.add_voxels(voxels)

    def __convert_voxels_to_opengl_format(self, voxels):
        voxels = np.array(voxels, np.float32)
        voxels = voxels.reshape(-1, 7)
        formatted_voxels = np.zeros(shape=(len(voxels), 6 * 8))

        for i in range(8):
            formatted_voxels[:, i * 6 + 3:i * 6 + 6] = voxels[:, 4:7]

        size = voxels[:, 3] / 2
        formatted_voxels[:, 0] = voxels[:, 0] - size  # x0
        formatted_voxels[:, 1] = voxels[:, 1] + size  # y0
        formatted_voxels[:, 2] = voxels[:, 2] - size  # z0

        formatted_voxels[:, 0 + 6] = voxels[:, 0] + size  # x1
        formatted_voxels[:, 1 + 6] = voxels[:, 1] + size  # y1
        formatted_voxels[:, 2 + 6] = voxels[:, 2] - size  # z1

        formatted_voxels[:, 0 + 6 * 2] = voxels[:, 0] + size  # x2
        formatted_voxels[:, 1 + 6 * 2] = voxels[:, 1] - size  # y2
        formatted_voxels[:, 2 + 6 * 2] = voxels[:, 2] - size  # z2

        formatted_voxels[:, 0 + 6 * 3] = voxels[:, 0] - size  # x3
        formatted_voxels[:, 1 + 6 * 3] = voxels[:, 1] - size  # y3
        formatted_voxels[:, 2 + 6 * 3] = voxels[:, 2] - size  # z3

        formatted_voxels[:, 0 + 6 * 4] = voxels[:, 0] - size  # x4
        formatted_voxels[:, 1 + 6 * 4] = voxels[:, 1] + size  # y4
        formatted_voxels[:, 2 + 6 * 4] = voxels[:, 2] + size  # z4

        formatted_voxels[:, 0 + 6 * 5] = voxels[:, 0] + size  # x5
        formatted_voxels[:, 1 + 6 * 5] = voxels[:, 1] + size  # y5
        formatted_voxels[:, 2 + 6 * 5] = voxels[:, 2] + size  # z5

        formatted_voxels[:, 0 + 6 * 6] = voxels[:, 0] + size  # x6
        formatted_voxels[:, 1 + 6 * 6] = voxels[:, 1] - size  # y6
        formatted_voxels[:, 2 + 6 * 6] = voxels[:, 2] + size  # z6

        formatted_voxels[:, 0 + 6 * 7] = voxels[:, 0] - size  # x7
        formatted_voxels[:, 1 + 6 * 7] = voxels[:, 1] - size  # y7
        formatted_voxels[:, 2 + 6 * 7] = voxels[:, 2] + size  # z7
        return formatted_voxels.flatten()

    def add_voxels(self, voxels):
        '''
        Adds voxels to renderers voxel data
        :param voxels: List of voxels where each voxel is of form [x, y, z, size, r, g b]
        :return: None
        '''
        voxels = self.__convert_voxels_to_opengl_format(voxels)
        old_voxels = self.__voxel_data.data

        voxels = self.__convert_point_data_z_axis(voxels)
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

    def clear_voxels(self):
        '''
        Clears renderers voxel data
        :return: None
        '''
        self.__voxel_data.set_array(np.array([], np.float32))

    def clear_all(self):
        '''
        Clears renderers voxel, line and point data
        :return: None
        '''
        self.clear_voxels()
        self.clear_lines()
        self.clear_points()

    @staticmethod
    def __convert_2d_bbox_to_depthrenderer_format(bbox):
        '''
        Takes a 2D bounding box of format [left, top, right, bottom] and converts it to DepthRenderer format of
        [center_x, center_y, half_width, half_height]
        :param bbox: 4 element list of format [left, top, right, bottom]
        :return: bbox in DepthRenderer format [center_x, center_y, half_width, half_height]
        '''

        half_width = (bbox[2] - bbox[0]) / 2
        half_height = (bbox[3] - bbox[1]) / 2

        center_x = (bbox[2] + bbox[0]) / 2
        center_y = (bbox[3] + bbox[1]) / 2

        return [center_x, center_y, half_width, half_height]

    def add_2d_bbox(self, bbox, principal_point, focal_length_in_pixels=715, color=[255, 255, 255], depth=10,
                    draw_3d=False, max_depth=80):
        '''
        Draws a 2d bbox on 3d space
        :param bbox: [left, top, right, bottom]
        :param principal_point: Center of the image
        :param focal_length_in_pixels: Camera's focal length measured in pixels
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :param depth: Depth of first bbox
        :param draw_3d: Draws a 2nd bbox whose location is based on max_depth argument
        :param max_depth: Depth of 2nd bbox, only effective when draw_3d is set to True
        :return: None
        '''
        color = np.array(color) / 256.0
        box_data = DepthRenderer.__convert_2d_bbox_to_depthrenderer_format(bbox)

        quad_data = [[box_data[0] - box_data[2], box_data[1] + box_data[3]],
                     [box_data[0] + box_data[2], box_data[1] + box_data[3]],
                     [box_data[0] + box_data[2], box_data[1] - box_data[3]],
                     [box_data[0] - box_data[2], box_data[1] - box_data[3]]]
        first_quad_points = np.array([Utils.convert_depth_pixel_to_point(x, y, depth, focal_length_in_pixels,
                                                                         principal_point, color)
                                      for x, y in quad_data])

        self.add_lines(np.array([first_quad_points[0], first_quad_points[1], first_quad_points[1], first_quad_points[2],
                                 first_quad_points[2], first_quad_points[3], first_quad_points[3],
                                 first_quad_points[0]]).flatten())

        if draw_3d:
            second_quad_points = np.array([Utils.convert_depth_pixel_to_point(x, y, max_depth, focal_length_in_pixels,
                                                                              principal_point, color)
                                           for x, y in quad_data])
            self.add_lines(
                np.array([second_quad_points[0], second_quad_points[1], second_quad_points[1], second_quad_points[2],
                          second_quad_points[2], second_quad_points[3], second_quad_points[3],
                          second_quad_points[0]]).flatten())

            self.add_lines(np.array([[first_quad_points[x], second_quad_points[x]] for x in range(4)]).flatten())

    def add_3d_object_bbox(self, bbox_data, color=[255, 255, 255]):
        '''
        Draws 3d bounding box with direction indicator
        :param bbox_data: [center_x, center_y, center_z, width, height, length, y_rotation]
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :return: None
        '''
        color = np.array(color) / 256.0
        first_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2],
                               [bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2]]

        second_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2]]

        matrix = glm.translate(glm.mat4(1), glm.vec3(*bbox_data[:3]))
        matrix = glm.rotate(matrix, bbox_data[-1], glm.vec3(0, 1, 0))

        first_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in first_rect_centered]
        second_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in second_rect_centered]

        front_point = list(matrix * glm.vec4(0, 0, bbox_data[5] * 0.8, 1))[:3]

        lines = [first_rect_centered[0], first_rect_centered[1],
                 first_rect_centered[1], first_rect_centered[2],
                 first_rect_centered[2], first_rect_centered[3],
                 first_rect_centered[3], first_rect_centered[0],
                 second_rect_centered[0], second_rect_centered[1],
                 second_rect_centered[1], second_rect_centered[2],
                 second_rect_centered[2], second_rect_centered[3],
                 second_rect_centered[3], second_rect_centered[0],
                 first_rect_centered[0], front_point,
                 first_rect_centered[1], front_point,
                 first_rect_centered[2], front_point,
                 first_rect_centered[3], front_point
                 ]
        lines = np.append(lines, np.array([[first_rect_centered[x], second_rect_centered[x]] for x
                                           in range(4)]).reshape(-1, 3), axis=0)
        lines = np.array([[x, y, z, color[0], color[1], color[2]] for x, y, z in lines])
        self.add_lines(lines)

    def add_3d_bbox(self, bbox_data, color=[255, 255, 255]):
        '''
        Draws 3d bounding box
        :param bbox_data: [center_x, center_y, center_z, width, height, length, y_rotation]
        :param color: A 3 element list that contains color [Red, Green, Blue], example = [255, 255, 255] for white
        :return: None
        '''
        color = np.array(color) / 256.0
        first_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, bbox_data[4] / 2, bbox_data[5] / 2],
                               [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2],
                               [bbox_data[3] / 2, -1 * bbox_data[4] / 2, bbox_data[5] / 2]]

        second_rect_centered = [[bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [-1 * bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2],
                                [bbox_data[3] / 2, -1 * bbox_data[4] / 2, -1 * bbox_data[5] / 2]]

        matrix = glm.translate(glm.mat4(1), glm.vec3(*bbox_data[:3]))
        matrix = glm.rotate(matrix, bbox_data[-1], glm.vec3(0, 1, 0))

        first_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in first_rect_centered]
        second_rect_centered = [list(matrix * glm.vec4(*vec, 1))[:3] for vec in second_rect_centered]

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
        '''
        Sets the camera location at given coordinates
        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :return: None
        '''
        self.__camera_coords = glm.vec3(x, y, -z)

    def move_forward(self, meters):
        '''
        Moves the camera forward towards the look direction by specified meters
        :param meters: Amount of meters to move
        :return:
        '''
        self.__camera_coords += self.__camera_front * meters

    def move_right(self, meters):
        '''
        Moves the camera to the right of the look direction by specified meters
        :param meters: Amount of meters to move
        :return:
        '''
        self.__camera_coords += glm.normalize(glm.cross(self.__camera_front, self.__camera_up)) * meters

    def look_up(self, angles):
        '''
        Makes the camera turn upwards by specified angle
        :param angles: Specifies how much to rotate camera
        :return:
        '''
        self.change_camera_rotation(0, angles)

    def look_right(self, angles):
        '''
        Makes the camera turn right by specified angle
        :param angles: Specifies how much to rotate camera
        :return:
        '''
        self.change_camera_rotation(angles, 0)

    def __process_input(self, frame_delay=0.016, process_movement=True):
        if glfw.get_key(self.__window, glfw.KEY_ESCAPE) == glfw.PRESS:
            current_time = time.time()
            if current_time - self.last_break_time > 0.5:
                self.break_render_loop = True
                self.last_break_time = current_time
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

        if glfw.get_key(self.__window, glfw.KEY_1) == glfw.PRESS:
            self.render_points = True
        if glfw.get_key(self.__window, glfw.KEY_2) == glfw.PRESS:
            self.render_points = False
        if glfw.get_key(self.__window, glfw.KEY_3) == glfw.PRESS:
            self.render_lines = True
        if glfw.get_key(self.__window, glfw.KEY_4) == glfw.PRESS:
            self.render_lines = False
        if glfw.get_key(self.__window, glfw.KEY_5) == glfw.PRESS:
            self.render_voxels = True
        if glfw.get_key(self.__window, glfw.KEY_6) == glfw.PRESS:
            self.render_voxels = False

    def add_voxel_map(self, voxel_map, voxel_map_center, voxel_size, filter_empty_voxels=True):
        '''
        Adds given voxel map to renderers voxel data
        :param voxel_map: A 3D array in x-y-z format where each element is list of size 3 that represents color
        :param voxel_map_center: Center of the voxel map
        :param voxel_size: Size of voxels in meters
        :param filter_empty_voxels: Filters out empty voxels if enabled
        :return: None
        '''
        x, y, z = np.meshgrid(np.arange(0, voxel_map.shape[0]), np.arange(0, voxel_map.shape[1]),
                              np.arange(0, voxel_map.shape[2]), indexing='ij')
        xyz_coordinates = np.array([x.flatten(), y.flatten(), z.flatten()], np.float32).T
        xyz_coordinates[:, 0] -= voxel_map.shape[0] / 2
        xyz_coordinates[:, 1] -= voxel_map.shape[1] / 2
        xyz_coordinates[:, 2] -= voxel_map.shape[2] / 2

        xyz_coordinates *= voxel_size
        xyz_coordinates += voxel_size / 2

        xyz_coordinates += voxel_map_center
        voxel_data = np.zeros(shape=(voxel_map.shape[0] * voxel_map.shape[1] * voxel_map.shape[2], 7))
        flattened_map = voxel_map.reshape(-1, 7)
        voxel_data[:, :3] = xyz_coordinates
        voxel_data[:, 3] = voxel_size
        voxel_data[:, 4:] = flattened_map[:, :3]

        if filter_empty_voxels:
            voxel_data = voxel_data[voxel_data[:, -1] != 0]

        self.add_voxels(voxel_data.flatten())

    def render(self, enable_controls=False):
        '''
        Renders existing points, lines and voxels to frame
        :param enable_controls: If true, camera will be allowed to move using keyboard
        :return: None
        '''
        current_time = time.time()
        self.__process_input(current_time - self.last_render_time, enable_controls)
        self.last_render_time = current_time

        GL.glClearColor(0.2, 0.3, 0.3, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.__update_shader_parameters()
        if self.render_points:
            self.__render_points()
        if self.render_lines:
            self.__render_lines()
        if self.render_voxels:
            self.__render_voxels()
        glfw.swap_buffers(self.__window)
        glfw.poll_events()

    def should_continue_rendering(self):
        '''
        A function for custom loops, custom loops should stop when this method returns false
        :return: boolean
        '''
        return not glfw.window_should_close(self.__window)

    def get_rendered_frame(self):
        '''
        Extracts image from renderers window
        :return: A numpy array that contains RGB frame on shape [frame_height, frame_width, 3]
        '''
        GL.glReadBuffer(GL.GL_FRONT)
        data = GL.glReadPixels(0, 0, self.frame_width, self.frame_height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        image = np.frombuffer(data, dtype=np.uint8).reshape(self.frame_height, self.frame_width, 3)
        image = np.flip(image, 0)
        return image

    def render_loop(self, enable_controls=True):
        '''
        A simple render loop, mainly used for controlling camera, loop will exit when user pressed ESC if controls are
        enabled
        :param enable_controls: If true, camera will be allowed to move using keyboard
        :return: None
        '''
        if self.is_window_hidden:
            self.was_window_hidden = True
            self.show_window()
        while self.should_continue_rendering() and not self.break_render_loop:
            self.render(enable_controls)
        if self.break_render_loop:
            self.break_render_loop = False

        if self.was_window_hidden:
            self.hide_window()
            self.was_window_hidden = False

    def close(self):
        '''
        Used for closing renderer
        :return: None
        '''
        glfw.terminate()

    def change_camera_rotation(self, yaw_offset, pitch_offset):
        '''
        Turns the camera by specified yaw and pitch values
        :param yaw_offset: Yaw value in degrees
        :param pitch_offset: Pitch value in degrees
        :return: None
        '''
        self.camera_yaw += yaw_offset
        self.camera_pitch += pitch_offset
        self.camera_pitch = np.clip(self.camera_pitch, -89, 89)
        direction = glm.vec3()
        direction.x = math.cos(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        direction.y = math.sin(glm.radians(self.camera_pitch))
        direction.z = math.sin(glm.radians(self.camera_yaw)) * math.cos(glm.radians(self.camera_pitch))
        self.__camera_front = glm.normalize(direction)
>>>>>>> master
