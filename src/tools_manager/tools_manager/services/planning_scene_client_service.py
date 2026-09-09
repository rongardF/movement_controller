# Copyright (c) 2026, Endtools Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""PlanningSceneClient — attach/detach the dispenser collision object in MoveIt."""

from __future__ import annotations

import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

from endtools.exceptions import ServiceUnavailableError
from endtools.models import DispenserConfigDTO
from endtools.utils import load_mesh_from_uri


class PlanningSceneClientService:

    def __init__(
        self,
        node: LifecycleNode,
        get_scene_service: str = '/get_planning_scene',
        apply_scene_service: str = '/apply_planning_scene',
        service_timeout_sec: float = 5.0,
    ) -> None:
        """Create the planning-scene service clients.

        :param node: Node used to create clients and access the logger/clock.
        :param get_scene_service: ``GetPlanningScene`` service name.
        :param apply_scene_service: ``ApplyPlanningScene`` service name.
        :param service_timeout_sec: Timeout for service availability and results.
        """
        self._node = node
        self._logger = node.get_logger()
        self._service_timeout_sec = service_timeout_sec
        self._callback_group = MutuallyExclusiveCallbackGroup()

        self._get_client = node.create_client(
            GetPlanningScene,
            get_scene_service,
            callback_group=self._callback_group,
        )
        self._apply_client = node.create_client(
            ApplyPlanningScene,
            apply_scene_service,
            callback_group=self._callback_group,
        )

    def allow_collisions(self, tool_sn: str, allowed: bool) -> bool:
        raise NotImplementedError()
        # TODO: allow/disallow collisions between tool_mount, tool-rack and the tool itself in the planning scene. 
        # This is necessary to allow the tool-mount to move into the tool-rack without collision checking errors.

    def attach_to_tool_mount(self, tool_sn: str) -> bool:
        raise NotImplementedError()
        # TODO: there should be an attahce object in planning scene; that object is attached to tool-rack
        # and it needs to be detached from tool-rack and then attahced to tool-mount; not sure if we have to remove
        # the object in planing scene or we can just modify it to be attached to tool-mount link

    def attach_to_tool_rack(self, tool_sn: str) -> bool:
        raise NotImplementedError()
        # TODO: there should be an attahce object in planning scene; that object is attached to tool-mount
        # and it needs to be detached from tool-mount and then attahced to tool-rack; not sure if we have to remove
        # the object in planing scene or we can just modify it to be attached to tool-rack link

    def _call(self, client, request, description: str):
        """Call a service and wait for the result within the configured timeout.

        Uses ``call_async`` and polls the future so this can run on an executor
        thread without nested spinning (the response is serviced by another
        thread of the node's ``MultiThreadedExecutor``).

        :param client: Service client to call.
        :param request: Request message.
        :param description: Human-readable service name for error messages.
        :returns: The service response.
        :raises ServiceUnavailableError: If unavailable or the call times out.
        """
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            message = f'Service {description} not available'
            self._logger.error(message)
            raise ServiceUnavailableError(message)

        future = client.call_async(request)
        deadline = time.monotonic() + self._service_timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                message = f'Service {description} call timed out'
                self._logger.error(message)
                raise ServiceUnavailableError(message)
            time.sleep(0.01)

        response = future.result()
        if response is None:
            message = f'Service {description} returned no result'
            self._logger.error(message)
            raise ServiceUnavailableError(message)
        return response
