import os
from pinocchio.robot_wrapper import RobotWrapper
import pinocchio as pin

def loadSharework(joints):


    root = os.path.dirname(os.path.realpath(__file__))
    urdf = os.path.join(root, "sharework.urdf")

    full_model_wrapper = RobotWrapper.BuildFromURDF(
        urdf,
        package_dirs=[root, os.path.join(root, "assets", "pkg")],  # <- add this
        root_joint=None,
    )
    full_model = full_model_wrapper.model  # Oggetto “Model” di Pinocchio



    # Neutral configuration of the full model
    q0 = pin.neutral(full_model)
    # q0 = np.array([0]*full_model.nq)

    # Find the IDs of the joints to KEEP
    keep_joint_ids = [full_model.getJointId(name) for name in joints]

    # Build the list of joints to LOCK (all others except universe)
    locked_joints = [
        j for j in range(1, full_model.njoints)  # skip universe (joint 0)
        if j not in keep_joint_ids
    ]

    # Create reduced model
    visual_model = full_model_wrapper.visual_model
    collision_model = full_model_wrapper.collision_model
    geom_models = [visual_model, collision_model]
    model, geometric_models_reduced = pin.buildReducedModel(full_model,
                                                            list_of_geom_models=geom_models,
                                                            list_of_joints_to_lock=locked_joints,
                                                            reference_configuration=q0,
                                                            )
    visual_model, collision_model = (
        geometric_models_reduced[0],
        geometric_models_reduced[1],
    )

    model_wrapper = full_model_wrapper

    print(f"qui!!")

    model_wrapper.model = model
    model_wrapper.visual_model = visual_model
    model_wrapper.collision_model = collision_model

    return model_wrapper

