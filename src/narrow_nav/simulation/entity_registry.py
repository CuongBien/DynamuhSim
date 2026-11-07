"""Domain names and categories independent of native geometry indices."""

from dataclasses import dataclass

from narrow_nav.core.validation import unique


@dataclass(frozen=True, slots=True)
class EntityRegistry:
    robot: str
    humans: tuple[str, ...]
    walls: tuple[str, ...]

    def __post_init__(self) -> None:
        unique((self.robot, *self.humans, *self.walls), "scene entity and wall IDs")

    def kind(self, entity_id: str) -> str:
        if entity_id == self.robot:
            return "robot"
        if entity_id in self.humans:
            return "human"
        if entity_id in self.walls:
            return "wall"
        raise KeyError(entity_id)
