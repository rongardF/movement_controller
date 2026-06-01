# Copyright (c) 2026, Movement Controller Contributors
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
"""TrajectoryGrouper — groups trajectory paths into blended execution groups."""

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class TrajectoryGrouper:
    """Stateless utility that implements the D-07 blend grouping algorithm."""

    @staticmethod
    def group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]:
        """Group trajectory paths into blended execution groups.

        Algorithm:
        - First path with blend_radius >= 0 starts a new group, subsequent paths with
          blend_radius > 0 are grouped with the previous group.
        - Any subsequent path with blend_radius <= 0 starts a new group.

        Args:
            paths: Non-empty list of validated TrajectoryPathDTO objects with unique
                   path_id values (guaranteed by TrajectoryGoalDTO validation).

        Returns:
            List of groups, where each group is a list of TrajectoryPathDTO objects.

        Raises:
            ValueError: If paths is empty.
        """
        if not paths:
            raise ValueError('paths list must not be empty')

        # Grouping loop
        groups: list[list[TrajectoryPathDTO]] = []
        for i, path in enumerate(paths):
            if i == 0:
                groups.append([path])
            elif path.blend_radius <= 0 and groups[-1][-1].blend_radius > 0:
                groups[-1].append(path)
            elif path.blend_radius <= 0 and groups[-1][-1].blend_radius <= 0:
                groups.append([path])
            elif path.blend_radius > 0 and groups[-1][-1].blend_radius <= 0:
                groups.append([path])
            else:
                groups[-1].append(path)

        return groups
