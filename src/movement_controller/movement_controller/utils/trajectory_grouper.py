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

        Algorithm (D-07):
        - The first path always starts a new group.
        - Any subsequent path with blend_radius > 0 is merged into the current group.
        - Any subsequent path with blend_radius <= 0 starts a new group.

        Args:
            paths: Non-empty list of validated TrajectoryPathDTO objects.

        Returns:
            List of groups, where each group is a list of TrajectoryPathDTO objects.

        Raises:
            ValueError: If paths is empty or contains duplicate path_id values.
        """
        if not paths:
            raise ValueError('paths list must not be empty')

        # Pre-validation: reject duplicate path_id values
        seen_ids: set[str] = set()
        for path in paths:
            if path.path_id in seen_ids:
                raise ValueError(f'Duplicate path_id: {path.path_id!r}')
            seen_ids.add(path.path_id)

        # Grouping loop
        groups: list[list[TrajectoryPathDTO]] = []
        for i, path in enumerate(paths):
            if i == 0 or path.blend_radius <= 0:
                groups.append([path])
            else:
                groups[-1].append(path)

        return groups
