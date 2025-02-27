import os
from ruamel.yaml import YAML


def remove_cylinder_objects(directory):
    yaml = YAML()
    yaml.preserve_quotes = True

    for i in range(1, 101):  # Iterating over scene0001.yaml to scene0100.yaml
        filename = f"scene{i:04d}.yaml"
        filepath = os.path.join(directory, filename)

        if not os.path.exists(filepath):
            continue

        with open(filepath, 'r') as file:
            data = yaml.load(file)

        if "world" in data and "collision_objects" in data["world"]:
            data["world"]["collision_objects"] = [
                obj for obj in data["world"]["collision_objects"]
                if not any(prim.get("type") == "cylinder" for prim in obj.get("primitives", []))
            ]

        with open(filepath, 'w') as file:
            yaml.dump(data, file)


# Example usage
directory = "/home/gaussian/harmonics/ros_ws/src/motion_bench_maker/problems/table_under_pick_panda_nocyl"
remove_cylinder_objects(directory)
