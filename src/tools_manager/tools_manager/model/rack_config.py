from pydantic import BaseModel, Field, model_validator

from endtools.enumerators.tool_type_enum import ToolTypeEnum


Pose6 = tuple[float, float, float, float, float, float]

class ToolSlotDTO(BaseModel):
    """A single tool slot defined on the rack."""

    name: str = Field(description='Rack slot identifier.')
    tool_sn: str = Field(description='Serial number of the tool assigned to the slot.')
    tool_lifted_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) of the tool lifted off the rack.')
    tool_attached_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) where tool-mount is fully inserted into the tool.')
    tool_slide_in_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) of from where the tool-mount is slid into the tool.')
    launch_file: str = Field(description='Launch file name that launches the tool node(s).')


class SimBootupDTO(BaseModel):
    """A tool spawned into simulation at bootup."""

    name: str = Field(description='Rack slot identifier.')
    tool_sn: str = Field(description='Serial number of the tool to spawn.')
    tool_type: ToolTypeEnum = Field(description='Type of the tool to spawn.')
    tool_part_number: str = Field(description='Part number of the tool to spawn.')
    tool_part_revision: str = Field(description='Part revision of the tool to spawn.')
    material_part_number: str = Field(description='Part number of the material used by the tool.')
    material_part_revision: str = Field(description='Part revision of the material used by the tool.')
    xacro_file: str = Field(description='Xacro description file for the tool.')


class RackConfigDTO(BaseModel):
    """Tool rack configuration: slot definitions and simulation bootup tools."""

    slots: list[ToolSlotDTO] = Field(description='Tool slots defined on the rack.')
    sim_bootup: list[SimBootupDTO] = Field(description='Tools spawned into simulation at bootup.')

    @model_validator(mode='after')
    def _no_duplicate_serial_numbers(self) -> 'RackConfigDTO':
        """Reject duplicate tool serial numbers within slots and within sim_bootup."""
        for section, entries in (('slots', self.slots), ('sim_bootup', self.sim_bootup)):
            seen: set[str] = set()
            duplicates = {e.tool_sn for e in entries if e.tool_sn in seen or seen.add(e.tool_sn)}
            if duplicates:
                raise ValueError(f'duplicate tool_sn in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _sim_bootup_matches_slots(self) -> 'RackConfigDTO':
        """Every slot must have exactly one matching sim_bootup entry (by slot name)."""
        slot_names = {slot.name for slot in self.slots}
        bootup_names = {bootup.name for bootup in self.sim_bootup}

        missing = slot_names - bootup_names
        if missing:
            raise ValueError(f'missing sim_bootup entries for slots: {sorted(missing)}')

        extra = bootup_names - slot_names
        if extra:
            raise ValueError(f'sim_bootup entries with no matching slot: {sorted(extra)}')

        return self

    def get_matched_tools(self) -> zip[tuple[ToolSlotDTO|None, SimBootupDTO|None]]:
        """Return slots and sim_bootup entries aligned pairwise by slot name.

        If a corresponding entry is missing in either list, None is returned for that entry.
        """
        names = [slot.name for slot in self.slots]
        names += [bootup.name for bootup in self.sim_bootup if bootup.name not in names]
        bootup_by_name = {bootup.name: bootup for bootup in self.sim_bootup}
        slots_by_name = {slot.name: slot for slot in self.slots}
        matched_slots: list[ToolSlotDTO|None] = []
        matched_bootups: list[SimBootupDTO|None] = []
        for name in names:
            matched_slots.append(slots_by_name.get(name, None))
            matched_bootups.append(bootup_by_name.get(name, None))

        return zip(matched_slots, matched_bootups)