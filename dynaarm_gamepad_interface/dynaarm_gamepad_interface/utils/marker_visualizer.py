#!/usr/bin/env python3

# Copyright 2025 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from dataclasses import dataclass
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from typing import List, Optional

@dataclass
class MarkerProperties:
    type: int
    scale_x: float = 0.05
    scale_y: float = 0.05
    scale_z: float = 0.05
    color_r: float = 1.0
    color_g: float = 0.0
    color_b: float = 0.0
    color_a: float = 1.0
    lifetime: float = 0.0

@dataclass
class MarkerData:
    id: int
    pose: PoseStamped
    frame: str
    properties: MarkerProperties

class MarkerHelper:

    def __init__(self, node, namespace_prefix: str = ""):
        self.node = node
        self.marker_publisher = self.node.create_publisher(MarkerArray, "/marker_visu", 100)
        self.namespace_prefix = namespace_prefix

    def publish_markers(self, marker_data_list: List[MarkerData]):
        """Publishes markers from a list of MarkerData objects."""
        marker_array = MarkerArray()

        for marker_data in marker_data_list:
            marker = Marker()
            marker.header.frame_id = marker_data.frame
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = f"{self.namespace_prefix}target_frame_{marker_data.id}"
            marker.id = marker_data.id
            marker.type = marker_data.properties.type
            marker.action = Marker.ADD
            marker.pose = marker_data.pose.pose
            marker.scale.x = marker_data.properties.scale_x
            marker.scale.y = marker_data.properties.scale_y
            marker.scale.z = marker_data.properties.scale_z
            marker.color.a = marker_data.properties.color_a
            marker.color.r = marker_data.properties.color_r
            marker.color.g = marker_data.properties.color_g
            marker.color.b = marker_data.properties.color_b            
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)
        self.node.get_logger().debug(f"Published {len(marker_data_list)} Markers to RViz.")

    def clear_markers(self, ids: Optional[List[int]] = None):
        """Clears all or specific markers."""
        marker_array = MarkerArray()
        if ids is None:
            # Clear all markers
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
        else:
            # Clear specific markers
            for id_to_clear in ids:
                marker = Marker()
                marker.id = id_to_clear
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
