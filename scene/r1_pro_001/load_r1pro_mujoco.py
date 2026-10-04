import mujoco
import mujoco.viewer


model = mujoco.MjModel.from_xml_path("r1_pro_001.xml")

data = mujoco.MjData(model)
with mujoco.viewer.launch(model, data):
    pass
