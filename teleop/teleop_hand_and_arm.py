import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper, G1DAgvClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from play_beep import play_beep, play_arpegio  # pribavoj
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
RECORD_DISCARD = False  # Discard current recording and return to READY state pribavoj
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

CONTROLLER_BUTTON_ACTIONS = {
    # Pico/Quest-style controller labels: left A/B are usually shown as X/Y.
    "left_ctrl_aButton": ("r", "left X"),
    "left_ctrl_bButton": ("s", "left Y"),
    "right_ctrl_aButton": ("q", "right A"),
    "right_ctrl_bButton": ("d", "right B"),  # pribavoj
}

def handle_control_key(key, source="keyboard"):
    global STOP, START, RECORD_TOGGLE, RECORD_DISCARD
    if key == 'r':
        START = True
        logger_mp.info(f"[{source}] start teleop requested.")
    elif key == 'q':
        START = False
        STOP = True
        logger_mp.info(f"[{source}] stop teleop requested.")
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
        logger_mp.info(f"[{source}] recording toggle requested.")
    elif key == 's':
        logger_mp.warning(f"[{source}] recording toggle ignored because teleop has not started.")
    elif key == 'd' and RECORD_RUNNING:  # pribavoj
        RECORD_DISCARD = True  # pribavoj
        logger_mp.info(f"[{source}] discard recording requested.")  # pribavoj
    else:
        logger_mp.warning(f"[{source}] {key} was pressed, but no action is defined for this key.")

def on_press(key):
    handle_control_key(key)

class ControllerButtonMapper:
    def __init__(self):
        self.previous_buttons = {button_name: False for button_name in CONTROLLER_BUTTON_ACTIONS}

    def update(self, tele_data):
        for button_name, (key, label) in CONTROLLER_BUTTON_ACTIONS.items():
            is_pressed = bool(getattr(tele_data, button_name, False))
            was_pressed = self.previous_buttons[button_name]
            if is_pressed and not was_pressed:
                handle_control_key(key, source=f"Pico {label}")
            self.previous_buttons[button_name] = is_pressed

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }

def clip_unit(value):
    return max(-1.0, min(1.0, float(value)))

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))

def apply_deadband(value, deadband):
    value = clip_unit(value)
    deadband = clamp(deadband, 0.0, 1.0)
    if abs(value) <= deadband:
        return 0.0
    return value

def controller_analog_button_value(is_pressed, value):
    value = clamp(value, 0.0, 1.0)
    if value == 0.0 and is_pressed:
        return 1.0
    return value

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    # network parameters
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--motion-base', type=str, choices=['loco', 'g1d_agv'], default='loco', help='Select motion-control backend')
    parser.add_argument('--motion-max-vx', type=float, default=0.2, help='Max forward velocity from controller stick, m/s')
    parser.add_argument('--motion-max-vy', type=float, default=0.3, help='Max lateral velocity from controller stick, m/s')
    parser.add_argument('--motion-max-vyaw', type=float, default=0.3, help='Max yaw velocity from controller stick, rad/s')
    parser.add_argument('--g1d-column-scale', type=float, default=0.5, help='Max normalized G1-D column velocity from right stick Y')
    parser.add_argument('--g1d-left-stick-x-mode', type=str, choices=['base_yaw', 'torso_yaw'], default='base_yaw', help='For G1-D AGV: map left stick X to base yaw or torso yaw')
    parser.add_argument('--g1d-torso-yaw-input', type=str, choices=['none', 'left_stick_x', 'side_triggers'], default=None, help='For G1-D AGV: optional torso yaw input source')
    parser.add_argument('--torso-yaw-max-rate', type=float, default=0.9, help='Max torso yaw target rate, rad/s')
    parser.add_argument('--torso-yaw-limit', type=float, default=2.0, help='Symmetric torso yaw software limit for joystick control, rad')
    parser.add_argument('--torso-yaw-deadband', type=float, default=0.08, help='Controller deadband for torso yaw input')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--record-torso-yaw', action='store_true', help='Also store measured torso yaw and commanded torso yaw target in body recording data')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    try:
        if args.motion and args.motion_base == "g1d_agv" and args.input_mode != "controller":
            raise ValueError("--motion-base g1d_agv requires --input-mode controller.")
        if args.torso_yaw_max_rate is not None and args.torso_yaw_max_rate < 0.0:
            raise ValueError("--torso-yaw-max-rate must be non-negative.")
        if args.torso_yaw_limit < 0.0:
            raise ValueError("--torso-yaw-limit must be non-negative.")
        if args.torso_yaw_deadband < 0.0:
            raise ValueError("--torso-yaw-deadband must be non-negative.")
        if args.g1d_left_stick_x_mode == "torso_yaw":
            if args.g1d_torso_yaw_input is not None and args.g1d_torso_yaw_input != "left_stick_x":
                raise ValueError("--g1d-left-stick-x-mode=torso_yaw conflicts with --g1d-torso-yaw-input.")
            g1d_torso_yaw_input = "left_stick_x"
        else:
            g1d_torso_yaw_input = args.g1d_torso_yaw_input or "none"
        arm_motion_mode = args.motion and args.motion_base != "g1d_agv"

        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     arm_reference_mode="head_yaw"
                                     )
        controller_button_mapper = ControllerButtonMapper() if args.input_mode == "controller" else None
        
        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                if args.motion_base == "g1d_agv":
                    loco_wrapper = G1DAgvClientWrapper()
                    logger_mp.info("Motion backend: G1-D AGV; arm commands stay on rt/lowcmd.")
                else:
                    loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=arm_motion_mode, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=arm_motion_mode, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=arm_motion_mode, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(motion_mode=arm_motion_mode, simulation_mode=args.sim)
        if args.record_torso_yaw and not hasattr(arm_ctrl, "get_current_waist_yaw"):
            raise ValueError("--record-torso-yaw is currently supported only with --arm=G1_29.")

        torso_yaw_target = 0.0
        torso_yaw_min = -abs(args.torso_yaw_limit)
        torso_yaw_max = abs(args.torso_yaw_limit)
        torso_yaw_max_rate = args.torso_yaw_max_rate if args.torso_yaw_max_rate is not None else args.motion_max_vyaw
        if args.motion and args.motion_base == "g1d_agv":
            if g1d_torso_yaw_input != "none":
                if not hasattr(arm_ctrl, "ctrl_waist_yaw"):
                    raise ValueError("--g1d-torso-yaw-input is currently supported only with --arm=G1_29.")
                waist_yaw_min, waist_yaw_max = arm_ctrl.get_waist_yaw_limits()
                torso_yaw_center = arm_ctrl.get_current_waist_yaw()
                torso_yaw_min = max(torso_yaw_center - abs(args.torso_yaw_limit), waist_yaw_min)
                torso_yaw_max = min(torso_yaw_center + abs(args.torso_yaw_limit), waist_yaw_max)
                torso_yaw_target = clamp(torso_yaw_center, torso_yaw_min, torso_yaw_max)
                arm_ctrl.ctrl_waist_yaw(torso_yaw_target)
                logger_mp.info(
                    f"G1-D mapping: {g1d_torso_yaw_input} controls torso yaw "
                    f"({torso_yaw_min:.2f} to {torso_yaw_max:.2f} rad from startup yaw {torso_yaw_center:.2f}); "
                    f"{'right' if g1d_torso_yaw_input == 'left_stick_x' else 'left'} stick X controls base yaw."
                )
            else:
                logger_mp.info("G1-D mapping: left stick X controls base yaw.")

        # end-effector
        xr_motion_data_ready = Value('b', False, lock=True)        # [input] whether XR hand/controller motion data has arrived
        if args.ee in ("dex3", "inspire_ftp", "inspire_dfx") and args.input_mode == "controller":
            raise ValueError(f"{args.ee} does not support controller input mode.")
        elif args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "hand":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_hand
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_hand(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                                dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "controller":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_ctrl
            left_gripper_trigger_in = Value('d', 10.0, lock=True)  # [input]
            left_gripper_squeeze_in = Value('d', 0.0, lock=True)   # [input]
            right_gripper_trigger_in = Value('d', 10.0, lock=True) # [input]
            right_gripper_squeeze_in = Value('d', 0.0, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_ctrl(left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                                                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] or Pico left X to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] or Pico left Y to START or SAVE recording (toggle cycle).")
            logger_mp.info("🟠  Press Pico right B to DISCARD the current recording.")  # pribavoj
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] or Pico right A to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            if controller_button_mapper is not None:
                tele_data = tv_wrapper.get_tele_data()
                controller_button_mapper.update(tele_data)

        if STOP:
            logger_mp.info("Stop requested before tracking started.")
            raise KeyboardInterrupt

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        arm_ctrl.speed_gradual_max()
        last_motion_time = time.time()

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            motion_dt = max(0.0, min(0.1, start_time - last_motion_time))
            last_motion_time = start_time
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # get xr's tele data and controller button actions
            tele_data = tv_wrapper.get_tele_data()
            if controller_button_mapper is not None:
                controller_button_mapper.update(tele_data)
            if STOP:
                break
            torso_yaw_input_value = 0.0

            if args.record and RECORD_DISCARD:  # pribavoj
                RECORD_DISCARD = False  # pribavoj
                play_beep(volume=0.001, duration=0.5)  # wake up audio, pribavoj
                play_arpegio(arpeggio = [392.00, 261.63])  # pribavoj

                if RECORD_RUNNING:  # pribavoj
                    RECORD_RUNNING = False  # pribavoj
                    recorder.discard_episode()  # pribavoj

                    if args.sim:  # pribavoj
                        publish_reset_category(1, reset_pose_publisher)  # pribavoj

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                        play_beep(volume=0.001, duration=0.5)  # wake up audio, pribavoj
                        play_arpegio(arpeggio = [261.63, 329.63, 392.00])  # pribavoj
    
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    play_beep(volume=0.001, duration=0.5)  # wake up audio, pribavoj
                    play_arpegio(arpeggio = [392.00, 329.63, 261.63])  # pribavoj
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            if args.ee in ("dex3", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "brainco" and args.input_mode == "controller":
                with left_gripper_trigger_in.get_lock():
                    left_gripper_trigger_in.value = tele_data.left_ctrl_triggerValue
                with left_gripper_squeeze_in.get_lock():
                    left_gripper_squeeze_in.value = tele_data.left_ctrl_squeezeValue
                with right_gripper_trigger_in.get_lock():
                    right_gripper_trigger_in.value = tele_data.right_ctrl_triggerValue
                with right_gripper_squeeze_in.get_lock():
                    right_gripper_squeeze_in.value = tele_data.right_ctrl_squeezeValue
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            with xr_motion_data_ready.get_lock():
                xr_motion_data_ready.value = tele_data.motion_data_ready
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                if args.motion_base == "g1d_agv":
                    if not (tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick):
                        vx = clip_unit(-tele_data.left_ctrl_thumbstickValue[1]) * args.motion_max_vx
                        if g1d_torso_yaw_input != "none":
                            if g1d_torso_yaw_input == "left_stick_x":
                                torso_yaw_axis = -tele_data.left_ctrl_thumbstickValue[0]
                                base_yaw_axis = -tele_data.right_ctrl_thumbstickValue[0]
                            else:
                                left_side = controller_analog_button_value(tele_data.left_ctrl_squeeze, tele_data.left_ctrl_squeezeValue)
                                right_side = controller_analog_button_value(tele_data.right_ctrl_squeeze, tele_data.right_ctrl_squeezeValue)
                                torso_yaw_axis = left_side - right_side
                                base_yaw_axis = -tele_data.left_ctrl_thumbstickValue[0]
                            torso_yaw_axis = apply_deadband(torso_yaw_axis, args.torso_yaw_deadband)
                            torso_yaw_input_value = torso_yaw_axis
                            torso_yaw_target = clamp(
                                torso_yaw_target + torso_yaw_axis * torso_yaw_max_rate * motion_dt,
                                torso_yaw_min,
                                torso_yaw_max,
                            )
                            arm_ctrl.ctrl_waist_yaw(torso_yaw_target)
                            vyaw = clip_unit(base_yaw_axis) * args.motion_max_vyaw
                        else:
                            vyaw = clip_unit(-tele_data.left_ctrl_thumbstickValue[0]) * args.motion_max_vyaw
                        column_vz = clip_unit(-tele_data.right_ctrl_thumbstickValue[1]) * args.g1d_column_scale
                        loco_wrapper.Move(vx, 0.0, vyaw)
                        loco_wrapper.HeightAdjust(column_vz)
                else:
                    loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * args.motion_max_vx,
                                      -tele_data.left_ctrl_thumbstickValue[0] * args.motion_max_vy,
                                      -tele_data.right_ctrl_thumbstickValue[0] * args.motion_max_vyaw)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif (args.ee == "brainco" and args.input_mode == "controller"):
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    body_state = {
                        "qpos": current_body_state,
                    }
                    body_action = {
                        "qpos": current_body_action,
                    }
                    if args.record_torso_yaw:
                        measured_torso_yaw = arm_ctrl.get_current_waist_yaw()
                        body_state["torso_yaw"] = measured_torso_yaw
                        body_action["torso_yaw_target"] = (
                            torso_yaw_target if g1d_torso_yaw_input != "none" else measured_torso_yaw
                        )
                        body_action["torso_yaw_input"] = torso_yaw_input_value
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": body_state, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": body_action, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")

        try:
            if args.motion and 'loco_wrapper' in locals() and hasattr(loco_wrapper, "Stop"):
                loco_wrapper.Stop()
        except Exception as e:
            logger_mp.error(f"Failed to stop motion wrapper: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
