import cv2
import numpy as np
import os, json
import dearpygui.dearpygui as dpg

import sys
sys.path.append(".")
from CameraCalibration.CameraMapping import CameraMapping
from DDS.GUIReceiver import GUIReceiver

class GUIProcess:
    def __init__(self):
        
        with open('Config/robot_calibration.json', 'r') as f:
            self.config = json.load(f)
            self.config = {k: np.array(v, dtype=np.float64) for k,v in self.config.items()}
            self.config['pos_dst'] = self.config['pos_dst'] + self.config['offset']
            self.config['homography_inv'] = np.linalg.inv(self.config['homography'])

        self.dds_receiver = GUIReceiver(
            on_detections_update=self.update_detection_points,
            on_tracks_update=self.update_tracker_points,
            on_pick_points_update=self.update_pick_points,
            on_target_points_update=self.update_target_points,
            on_delta_position_update=self.update_delta_position,
            on_image_update=self.update_image,
        )

        
        dpg.create_context()
        #dpg.show_metrics()
        #dpg.configure_app(docking=True, docking_space=True, load_init_file="Config/camera_follow_layout.ini")
        dpg.create_viewport(title='Custom Title', width=1920, height=1080)
        dpg.setup_dearpygui()

        with dpg.theme(tag="detection_theme"):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (255,0,0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Circle, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)

        with dpg.theme(tag="tracker_theme"):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (0,255,0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Asterisk, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)

        with dpg.theme(tag="pick_point_theme"):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (0,0,255), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Cross, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 15, category=dpg.mvThemeCat_Plots)

        with dpg.theme(tag="target_theme"):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (0,255,0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Plus, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 15, category=dpg.mvThemeCat_Plots)

        with dpg.theme(tag="delta_position_theme"):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (255,255,0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Diamond, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)



        with dpg.window(label="Plot Window", tag="Primary Window"):

            frame_size = [int(self.config['frame_size'][1]), int(self.config['frame_size'][0])]
            # Create texture registry
            with dpg.texture_registry():
                self.texture_id = dpg.add_raw_texture(
                    frame_size[0], frame_size[1], np.zeros((frame_size[0], frame_size[1], 3)),
                    format=dpg.mvFormat_Float_rgb
                )
            
            # Add image plot
            with dpg.plot(label="Image Plot", width=1900, height=1000):
                dpg.add_plot_legend()
                xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="X")
                yaxis = dpg.add_plot_axis(dpg.mvYAxis, label="Y")

                dpg.set_axis_limits(xaxis, -1, 5)
                dpg.set_axis_limits(yaxis, 0, 2)
                
                # Add image series
                dpg.add_image_series(
                    self.texture_id,
                    [0, 0], [-1, 1],
                    parent=xaxis
                )
                
                # Add scatter series for points
                dpg.add_scatter_series(
                    [], [], parent=xaxis, label="Detection Points", tag="detection_scatter_series"
                )

                dpg.add_scatter_series(
                    [], [], parent=xaxis, label="Tracker Points", tag="tracker_scatter_series"
                )

                dpg.add_scatter_series(
                    [], [], parent=xaxis, label="Pick Points", tag="pick_point_scatter_series"
                )

                dpg.add_scatter_series(
                    [], [], parent=xaxis, label="Target Points", tag="target_scatter_series"
                )

                dpg.add_scatter_series(
                    [], [], parent=xaxis, label="Delta Position", tag="delta_position_scatter_series"
                )



                dpg.bind_item_theme("detection_scatter_series", "detection_theme")
                dpg.bind_item_theme("tracker_scatter_series", "tracker_theme")
                dpg.bind_item_theme("pick_point_scatter_series", "pick_point_theme")
                dpg.bind_item_theme("target_scatter_series", "target_theme")
                dpg.bind_item_theme("delta_position_scatter_series", "delta_position_theme")


        dpg.set_primary_window("Primary Window", True)
        dpg.show_viewport()

    def update_image(self, image):
        # Ensure image is float32 and in range [0, 1]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        image = cv2.flip(image, 0)
        if image.dtype != np.float32:
            image = image.astype(np.float32) / 255.0
        dpg.set_value(self.texture_id, image)

    def update_detection_points(self, points):
        if points.shape[0] == 0:
            dpg.set_value("detection_scatter_series", [[], []])
            return
        uv = CameraMapping.map_from_pixels_to_uv(points, self.config['frame_size'])
        y = np.ascontiguousarray(uv[:, 0])
        x = -np.ascontiguousarray(uv[:, 1])
        dpg.set_value("detection_scatter_series", [x, y])

    def update_tracker_points(self, points):
        if points.shape[0] == 0:
            dpg.set_value("detection_scatter_series", [[], []])
            return
        uv = CameraMapping.apply_homography(points, self.config['homography_inv'])
        y = np.ascontiguousarray(uv[:, 0])
        x = -np.ascontiguousarray(uv[:, 1])
        dpg.set_value("tracker_scatter_series", [x, y])

    def update_pick_points(self, points):
        if points.shape[0] == 0:
            dpg.set_value("pick_point_scatter_series", [[], []])
            return
        uv = CameraMapping.apply_homography(points, self.config['homography_inv'])
        y = np.ascontiguousarray(uv[:, 0])
        x = -np.ascontiguousarray(uv[:, 1])
        dpg.set_value("pick_point_scatter_series", [x, y])

    def update_target_points(self, points):
        if points.shape[0] == 0:
            dpg.set_value("target_scatter_series", [[], []])
            return
        uv = CameraMapping.apply_homography(points, self.config['homography_inv'])
        y = np.ascontiguousarray(uv[:, 0])
        x = -np.ascontiguousarray(uv[:, 1])
        dpg.set_value("target_scatter_series", [x, y])

    def update_delta_position(self, position):
        if position.shape[0] == 0:
            dpg.set_value("delta_position_scatter_series", [[], []])
            return
        uv = CameraMapping.apply_homography(position, self.config['homography_inv'])
        y = np.ascontiguousarray(uv[:, 0])
        x = -np.ascontiguousarray(uv[:, 1])
        dpg.set_value("delta_position_scatter_series", [x, y])


    def run(pipe_parent, pipe_child, gui_queue):
        gui = GUIProcess()
        while True:

            if pipe_parent.poll():
                msg = pipe_parent.recv()
                if msg[0] == "EXIT":
                    break
                elif msg[0] in gui.function_map:
                    gui.function_map[msg[0]](msg[1])
                else:
                    print(f"Invalid message: {msg}")

            while not gui_queue.empty():
                msg = gui_queue.get()
                if msg[0] in gui.function_map:
                    pass
                    #gui.function_map[msg[0]](msg[1])
                else:
                    print(f"Invalid message: {msg}")

            dpg.render_dearpygui_frame()

if __name__ == "__main__":
    gui = GUIProcess()

    try:
        while True:
            dpg.render_dearpygui_frame()
    except KeyboardInterrupt:
        print("Exiting GUI...")
        dpg.destroy_context()
        sys.exit(0)

    dpg.destroy_context()
    sys.exit(0)
