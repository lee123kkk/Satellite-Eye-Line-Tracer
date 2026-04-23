import math
import os

measurements = [
    ( 1.28,  1.71), (-0.27,  8.00), (-2.01, -0.14), (-0.98,  0.79),
    ( 2.88,  1.59), ( 0.00,  0.68), (-5.95,  0.00), (-0.08, -2.58),
    ( 0.11, -6.80), (-1.09, -3.87), ( 5.89,  0.00)
]

scale_factor = 0.1
current_x, current_y = 1.5, -1.5
points = [(current_x, current_y)]

for dx, dy in measurements:
    current_x += (dx * scale_factor)
    current_y += (dy * scale_factor)
    points.append((current_x, current_y))

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="track_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="ortho_camera_fix">
      <static>true</static>
      <pose>0.8 -0.8 4.5 0 1.5707 1.5707</pose> 
      <link name="link">
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.4</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <topic>/camera/image_raw</topic>
        </sensor>
      </link>
    </model>

    <model name="blue_tape_track">
      <static>true</static>
"""

for i in range(len(points)):
    x1, y1 = points[i]
    x2, y2 = points[(i + 1) % len(points)] 
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    length = math.hypot(x2 - x1, y2 - y1) + 0.06 
    angle = math.atan2(y2 - y1, x2 - x1)
    
    sdf_content += f"""
      <link name="line_{i}">
        <pose>{cx} {cy} 0.001 0 0 {angle}</pose>
        <visual name="visual">
          <geometry><box><size>{length} 0.05 0.002</size></box></geometry>
          <material>
            <ambient>0.1 0.3 0.8 1</ambient>
            <diffuse>0.1 0.3 0.8 1</diffuse>
          </material>
        </visual>
      </link>"""

sdf_content += """
    </model>
  </world>
</sdf>"""

file_path = os.path.expanduser('~/P1_ws/src/p1_pkg/worlds/track.world')
with open(file_path, 'w') as f:
    f.write(sdf_content)
