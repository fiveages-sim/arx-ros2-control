"""
最小化夹爪测试脚本 - 只控制夹爪开合，不移动关节
用法: python test_gripper_only.py X5 can1
"""
import time
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click


@click.command()
@click.argument("model")       # X5 or L5
@click.argument("interface")   # can1, can3 etc.
def main(model: str, interface: str):
    print(f"Connecting to {model} on {interface}...")
    arx5_joint_controller = arx5.Arx5JointController(model, interface)
    robot_config = arx5_joint_controller.get_robot_config()
    controller_config = arx5_joint_controller.get_controller_config()

    print(f"gripper_width: {robot_config.gripper_width}")
    print(f"gripper_open_readout: {robot_config.gripper_open_readout}")

    arx5_joint_controller.reset_to_home()
    print("Reset to home done.")

    # 读取初始状态
    state = arx5_joint_controller.get_joint_state()
    print(f"Initial gripper_pos: {state.gripper_pos:.4f}")

    dt = controller_config.controller_dt
    gripper_width = robot_config.gripper_width  # 0.088

    # --- 阶段 1: 逐渐打开夹爪 (0 -> gripper_width) ---
    print("\n=== Opening gripper ===")
    step_num = 500  # 1s
    for i in range(step_num):
        cmd = arx5.JointState(robot_config.joint_dof)
        # 关节保持零位
        cmd.gripper_pos = (i / (step_num - 1)) * gripper_width
        arx5_joint_controller.set_joint_cmd(cmd)
        time.sleep(dt)
        if i % 100 == 0:
            s = arx5_joint_controller.get_joint_state()
            print(f"  step {i}: cmd={cmd.gripper_pos:.4f}, actual={s.gripper_pos:.4f}")

    state = arx5_joint_controller.get_joint_state()
    print(f"After open: gripper_pos={state.gripper_pos:.4f} (target={gripper_width:.4f})")

    time.sleep(1.0)

    # --- 阶段 2: 逐渐关闭夹爪 (gripper_width -> 0) ---
    print("\n=== Closing gripper ===")
    for i in range(step_num):
        cmd = arx5.JointState(robot_config.joint_dof)
        cmd.gripper_pos = (1 - i / (step_num - 1)) * gripper_width
        arx5_joint_controller.set_joint_cmd(cmd)
        time.sleep(dt)
        if i % 100 == 0:
            s = arx5_joint_controller.get_joint_state()
            print(f"  step {i}: cmd={cmd.gripper_pos:.4f}, actual={s.gripper_pos:.4f}")

    state = arx5_joint_controller.get_joint_state()
    print(f"After close: gripper_pos={state.gripper_pos:.4f} (target=0.0000)")

    print("\nDone! Resetting to home...")
    arx5_joint_controller.reset_to_home()


if __name__ == "__main__":
    main()
