from os import path


def model_xml_path(default_path: str, control_type: str):  # TODO remove (unused)
    assert control_type is not None
    prefix = "franka_"
    directory = path.dirname(default_path)
    file_name = path.basename(default_path)
    assert file_name.startswith(prefix)

    if control_type == "direct":
        return path.join(directory, prefix + control_type + "_" + file_name[len(prefix):])
    else:
        return default_path


def direct_set_action(sim, action):
    assert action.shape == (9,)
    max_forces = sim.model.actuator_forcerange[:7, 1]

    if sim.data.ctrl is not None:
        # Revolute joints
        sim.data.ctrl[:7] = action[:7] * max_forces

        # Gripper
        for i in range(7, 9):
            idx = sim.model.jnt_qposadr[sim.model.actuator_trnid[i, 0]]
            sim.data.ctrl[i] = sim.data.qpos[idx] + action[i]

