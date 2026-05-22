from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import sys
import gc
import time
import ujson
import aidemo
import image
import nncase_runtime as nn
import ulab.numpy as np


# Yellow-ball detector parameters are kept aligned with
# C:\Users\15897\Desktop\yellow_ball_center_k230.py.
BALL_MODEL_PATH = "/sdcard/yolo11s_balldetect_3class_416.kmodel"
BALL_LABELS = ["green ball", "red ball", "yellow ball"]
YELLOW_CLASS_ID = 2

BALL_MODEL_INPUT_SIZE = [416, 416]
RGB888P_SIZE = [640, 360]
DISPLAY_MODE = "lcd"

BALL_CONF = 0.25
NMS_THRESH = 0.45
MAX_BOXES = 20

# Post filter in display-coordinate scale.
BALL_MIN_AREA_RATIO = 0.0008
BALL_MAX_AREA_RATIO = 0.35
BALL_MIN_ASPECT = 0.50
BALL_MAX_ASPECT = 2.00

# Calibrated with data/vision_calibration/CSV and PHOTOS.
# The upper area limit is provisional until too-close negative samples exist.
GRAB_CENTER_X = 323
GRAB_CENTER_Y = 404
CENTER_TOLERANCE_X = 45
CENTER_TOLERANCE_Y = 25
BALL_GRAB_AREA_MIN = 13000
BALL_GRAB_AREA_MAX = 17500
BALL_GRAB_SCORE_MIN = 0.70
BALL_STABLE_FRAME_MIN = 3
BALL_LOST_TIMEOUT_MS = 500

ENABLE_JSON_LOG = False
LOG_INTERVAL_MS = 100
DEBUG_MODE = 0

# MQTT config. Publish to the PC Mosquitto broker when enabled.
ENABLE_MQTT = False
MQTT_INTERVAL_MS = 500
MQTT_RECONNECT_MS = 15000
MQTT_CONNECT_AT_START = False
MQTT_SOCKET_TIMEOUT_S = 0.2
MQTT_DIAG_LOG = False
MQTT_DEVICE_ID = "k230_zone1_grab_window_001"
MQTT_TOPIC = b"k230/vision/ball_center"
MQTT_BROKER = "192.168.137.1"
MQTT_PORT = 1883
MQTT_USER = None
MQTT_PASSWORD = None
MQTT_KEEPALIVE = 30

# Offline calibration does not need WiFi. Leave these empty unless MQTT is used.
WIFI_SSID = ""
WIFI_PASSWORD = ""
WIFI_TIMEOUT_MS = 3000

# Local calibration log. Copy the CSV from the K230 SD card after testing.
ENABLE_CALIBRATION_FILE_LOG = True
CALIBRATION_LOG_PATH = "/sdcard/fire_car_vision_calibration.csv"
CALIBRATION_LOG_RESET_ON_START = False
CALIBRATION_LOG_EVERY_N_FRAMES = 5
CALIBRATION_LOG_FOUND_ONLY = True
CALIBRATION_SESSION = "zone1_pick"
ENABLE_CALIBRATION_IMAGE_LOG = True
CALIBRATION_IMAGE_DIR = "/sdcard/fire_car_vision_images"
CALIBRATION_IMAGE_QUALITY = 85

CALIBRATION_CSV_HEADER = (
    "sample_index,image_name,session,ts_ms,found,cx,cy,area,box_x,box_y,box_w,box_h,"
    "score,direction,center_ok,area_ok,grab_ready,stable_count,"
    "fps,display_w,display_h\n"
)


def align_up(value, align):
    return ((value + align - 1) // align) * align


def letterbox_pad_param(src_size, dst_size):
    src_w, src_h = src_size[0], src_size[1]
    dst_w, dst_h = dst_size[0], dst_size[1]
    ratio = min(float(dst_w) / float(src_w), float(dst_h) / float(src_h))
    new_w = int(round(src_w * ratio))
    new_h = int(round(src_h * ratio))
    dw = float(dst_w - new_w) / 2.0
    dh = float(dst_h - new_h) / 2.0
    top = 0
    bottom = int(round(dh * 2 + 0.1))
    left = 0
    right = int(round(dw * 2 - 0.1))
    return top, bottom, left, right, ratio


def clamp_xy(x, y, max_w, max_h):
    if x < 0:
        x = 0
    if y < 0:
        y = 0
    if x >= max_w:
        x = max_w - 1
    if y >= max_h:
        y = max_h - 1
    return x, y


def rect_dir(cx, total_w):
    if cx < total_w * 0.38:
        return "left"
    if cx > total_w * 0.62:
        return "right"
    return "center"


def pass_ball_shape_filter(x, y, w, h, display_size):
    if w <= 0 or h <= 0:
        return False

    area_ratio = float(w * h) / float(display_size[0] * display_size[1])
    aspect = float(w) / float(h)

    if area_ratio < BALL_MIN_AREA_RATIO or area_ratio > BALL_MAX_AREA_RATIO:
        return False
    if aspect < BALL_MIN_ASPECT or aspect > BALL_MAX_ASPECT:
        return False
    return True


def grab_center_configured():
    return GRAB_CENTER_X >= 0 and GRAB_CENTER_Y >= 0


def grab_area_configured():
    return BALL_GRAB_AREA_MIN >= 0 and BALL_GRAB_AREA_MAX >= 0 and BALL_GRAB_AREA_MAX >= BALL_GRAB_AREA_MIN


def area_state(area):
    if not grab_area_configured():
        return "not_configured"
    if area < BALL_GRAB_AREA_MIN:
        return "too_far"
    if area > BALL_GRAB_AREA_MAX:
        return "too_close"
    return "ok"


def add_grab_window_fields(event, stable_count):
    center_ok = False
    area_ok = False
    score_ok = False
    dx = None
    dy = None

    event["grab_window"] = {
        "center": [GRAB_CENTER_X, GRAB_CENTER_Y] if grab_center_configured() else None,
        "tolerance": [CENTER_TOLERANCE_X, CENTER_TOLERANCE_Y],
        "area_range": [BALL_GRAB_AREA_MIN, BALL_GRAB_AREA_MAX] if grab_area_configured() else None,
        "score_min": BALL_GRAB_SCORE_MIN,
        "stable_frame_min": BALL_STABLE_FRAME_MIN,
        "lost_timeout_ms": BALL_LOST_TIMEOUT_MS,
    }

    if event["center"] is not None and grab_center_configured():
        dx = event["center"][0] - GRAB_CENTER_X
        dy = event["center"][1] - GRAB_CENTER_Y
        center_ok = abs(dx) <= CENTER_TOLERANCE_X and abs(dy) <= CENTER_TOLERANCE_Y

    if "area" in event:
        area_ok = area_state(event["area"]) == "ok"

    if "score" in event:
        score_ok = event["score"] >= BALL_GRAB_SCORE_MIN

    event["offset"] = [dx, dy] if dx is not None and dy is not None else None
    event["center_ok"] = center_ok
    event["centered"] = center_ok
    event["area_ok"] = area_ok
    event["score_ok"] = score_ok
    event["area_state"] = area_state(event["area"]) if "area" in event else "not_found"
    event["frame_ready"] = center_ok and area_ok and score_ok
    event["stable_count"] = stable_count
    event["grab_ready"] = event["frame_ready"] and stable_count >= BALL_STABLE_FRAME_MIN
    return event


def build_ball_event(det, now_ms, stable_count, fps):
    x, y, w, h = det["box"]
    cx = x + w // 2
    cy = y + h // 2
    cx, cy = clamp_xy(cx, cy, det["display_size"][0], det["display_size"][1])

    area = int(w * h)
    event = {
        "device": MQTT_DEVICE_ID,
        "ts_ms": now_ms,
        "found": True,
        "status": "ok",
        "label": det["label"],
        "score": det["score"],
        "box": [x, y, w, h],
        "center": [cx, cy],
        "area": area,
        "display_size": det["display_size"],
        "direction": rect_dir(cx, det["display_size"][0]),
        "fps": fps,
    }

    return add_grab_window_fields(event, stable_count)


def empty_event(now_ms, display_size, stable_count, fps):
    event = {
        "device": MQTT_DEVICE_ID,
        "ts_ms": now_ms,
        "found": False,
        "status": "empty",
        "center": None,
        "box": None,
        "display_size": display_size,
        "fps": fps,
    }
    return add_grab_window_fields(event, stable_count)


def calibration_log_header_needed(path):
    try:
        stat = os.stat(path)
        return stat[6] == 0
    except Exception:
        return True


def ensure_dir(path):
    try:
        os.stat(path)
        return True
    except Exception:
        pass

    try:
        os.mkdir(path)
        return True
    except Exception as e:
        print("mkdir failed:", path, e)
        return False


def parent_dir_from_path(path):
    index = path.rfind("/")
    if index <= 0:
        return ""
    return path[:index]


def ensure_parent_dir(path):
    parent = parent_dir_from_path(path)
    if parent == "":
        return True
    return ensure_dir(parent)


def open_calibration_log():
    if not ENABLE_CALIBRATION_FILE_LOG:
        return None

    try:
        if not ensure_parent_dir(CALIBRATION_LOG_PATH):
            return None

        need_header = CALIBRATION_LOG_RESET_ON_START or calibration_log_header_needed(CALIBRATION_LOG_PATH)
        log_file = open(CALIBRATION_LOG_PATH, "w" if CALIBRATION_LOG_RESET_ON_START else "a")
        if need_header:
            log_file.write(CALIBRATION_CSV_HEADER)
            try:
                log_file.flush()
            except Exception:
                pass
        print("calibration log:", CALIBRATION_LOG_PATH)
        return log_file
    except Exception as e:
        print("calibration log disabled:", e)
        return None


def csv_bool(value):
    return "1" if value else "0"


def csv_value(value):
    if value is None:
        return ""
    return str(value)


def parse_sample_index_from_name(name):
    if not name.endswith(".jpg"):
        return 0
    stem = name[:-4]
    try:
        return int(stem)
    except Exception:
        return 0


def next_calibration_sample_index():
    next_index = 1

    try:
        for name in os.listdir(CALIBRATION_IMAGE_DIR):
            index = parse_sample_index_from_name(name)
            if index >= next_index:
                next_index = index + 1
    except Exception:
        pass

    return next_index


def should_write_calibration_sample(payload):
    if CALIBRATION_LOG_FOUND_ONLY and not payload.get("found"):
        return False
    return True


def try_save_image(image_obj, image_path):
    try:
        image_obj.save(image_path, quality=CALIBRATION_IMAGE_QUALITY)
        return True
    except TypeError:
        try:
            image_obj.save(image_path)
            return True
        except Exception:
            return False
    except Exception:
        return False


def save_converted_image(image_obj, image_path):
    try:
        rgb_img = image_obj.to_rgb888()
        if try_save_image(rgb_img, image_path):
            return True
    except Exception:
        pass

    try:
        rgb565_img = image_obj.to_rgb565()
        if try_save_image(rgb565_img, image_path):
            return True
    except Exception:
        pass

    return False


def ndarray_frame_size(array_obj):
    try:
        shape = array_obj.shape
        if len(shape) >= 4:
            return int(shape[3]), int(shape[2])
        if len(shape) == 3:
            return int(shape[2]), int(shape[1])
    except Exception:
        pass
    return align_up(RGB888P_SIZE[0], 16), RGB888P_SIZE[1]


def image_from_ndarray(array_obj):
    try:
        width, height = ndarray_frame_size(array_obj)
        raw_bytes = bytes(array_obj.flatten())
        return image.Image(
            width,
            height,
            image.RGBP888,
            alloc=image.ALLOC_REF,
            data=raw_bytes,
        )
    except Exception as e:
        print("calibration image wrap failed:", e)
        return None


def image_size_or_default(image_obj):
    try:
        return int(image_obj.width()), int(image_obj.height())
    except Exception:
        return align_up(RGB888P_SIZE[0], 16), RGB888P_SIZE[1]


def draw_payload_on_saved_image(image_obj, payload):
    box = payload.get("box")
    center = payload.get("center")
    display_size = payload.get("display_size")

    if box is None or center is None or display_size is None:
        return image_obj

    image_w, image_h = image_size_or_default(image_obj)
    if display_size[0] <= 0 or display_size[1] <= 0:
        return image_obj

    sx = float(image_w) / float(display_size[0])
    sy = float(image_h) / float(display_size[1])
    x = int(box[0] * sx)
    y = int(box[1] * sy)
    w = int(box[2] * sx)
    h = int(box[3] * sy)
    cx = int(center[0] * sx)
    cy = int(center[1] * sy)

    if w <= 0:
        w = 1
    if h <= 0:
        h = 1

    try:
        image_obj.draw_rectangle(x, y, w, h, color=(255, 220, 0), thickness=3)
        image_obj.draw_cross(cx, cy, color=(255, 0, 0), thickness=2)
        image_obj.draw_circle(cx, cy, 6, color=(255, 0, 0), thickness=2)
    except Exception as e:
        print("calibration image draw failed:", e)

    return image_obj


def save_image_object(image_obj, image_path, payload=None):
    if not hasattr(image_obj, "save"):
        frame_img = image_from_ndarray(image_obj)
        if frame_img is None:
            return False

        try:
            frame_img = frame_img.to_rgb888()
        except Exception:
            try:
                frame_img = frame_img.to_rgb565()
            except Exception:
                pass

        if payload is not None:
            frame_img = draw_payload_on_saved_image(frame_img, payload)

        if try_save_image(frame_img, image_path):
            return True

        print("calibration image save failed:", image_path, "unsupported image format")
        return False

    try:
        copied = image_obj.copy()
    except Exception:
        copied = image_obj

    if payload is not None:
        copied = draw_payload_on_saved_image(copied, payload)

    if try_save_image(copied, image_path):
        return True
    if save_converted_image(copied, image_path):
        return True

    print("calibration image save failed:", image_path, "unsupported image format")
    return False


def get_calibration_snapshot(pl):
    try:
        return pl.sensor.snapshot(chn=1)
    except TypeError:
        try:
            return pl.sensor.snapshot()
        except Exception as e:
            print("calibration snapshot failed:", e)
            return None
    except Exception as e:
        print("calibration snapshot failed:", e)
        return None


def save_calibration_image(pl, img, sample_index, payload):
    if not ENABLE_CALIBRATION_IMAGE_LOG:
        return ""

    if not ensure_dir(CALIBRATION_IMAGE_DIR):
        return ""

    image_name = "%d.jpg" % sample_index
    image_path = "%s/%s" % (CALIBRATION_IMAGE_DIR, image_name)

    snapshot = get_calibration_snapshot(pl)
    if snapshot is not None and save_image_object(snapshot, image_path, payload):
        return image_name

    if save_image_object(img, image_path, payload):
        return image_name

    return ""


def write_calibration_log(log_file, payload, sample_index, image_name):
    if log_file is None:
        return
    if not should_write_calibration_sample(payload):
        return

    center = payload.get("center")
    box = payload.get("box")
    display_size = payload.get("display_size")
    if center is None:
        center = ["", ""]
    if box is None:
        box = ["", "", "", ""]
    if display_size is None:
        display_size = ["", ""]

    try:
        line = "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            csv_value(sample_index),
            csv_value(image_name),
            CALIBRATION_SESSION,
            csv_value(payload.get("ts_ms")),
            csv_bool(payload.get("found")),
            csv_value(center[0]),
            csv_value(center[1]),
            csv_value(payload.get("area")),
            csv_value(box[0]),
            csv_value(box[1]),
            csv_value(box[2]),
            csv_value(box[3]),
            csv_value(payload.get("score")),
            csv_value(payload.get("direction")),
            csv_bool(payload.get("center_ok")),
            csv_bool(payload.get("area_ok")),
            csv_bool(payload.get("grab_ready")),
            csv_value(payload.get("stable_count")),
            csv_value(payload.get("fps")),
            csv_value(display_size[0]),
            csv_value(display_size[1]),
        )
        log_file.write(line)
        try:
            log_file.flush()
        except Exception:
            pass
    except Exception as e:
        print("calibration log write failed:", e)


def draw_ball(osd_img, det):
    x, y, w, h = det["box"]
    cx = x + w // 2
    cy = y + h // 2
    cx, cy = clamp_xy(cx, cy, det["display_size"][0], det["display_size"][1])
    title = "yellow %.2f (%d,%d)" % (det["score"], cx, cy)
    text_y = y - 28
    if text_y < 0:
        text_y = y + 2

    osd_img.draw_rectangle(x, y, w, h, color=(255, 210, 0), thickness=3)
    osd_img.draw_cross(cx, cy, color=(255, 0, 0), thickness=2)
    osd_img.draw_circle(cx, cy, 6, color=(255, 0, 0), thickness=2)
    osd_img.draw_string_advanced(x, text_y, 24, title, color=(255, 220, 0))


def draw_grab_center(osd_img):
    label_x = GRAB_CENTER_X + 8
    label_y = GRAB_CENTER_Y - 28

    if not grab_center_configured():
        return
    osd_img.draw_cross(GRAB_CENTER_X, GRAB_CENTER_Y, color=(255, 0, 255), thickness=2)
    osd_img.draw_circle(GRAB_CENTER_X, GRAB_CENTER_Y, 8, color=(255, 0, 255), thickness=2)
    osd_img.draw_rectangle(
        GRAB_CENTER_X - CENTER_TOLERANCE_X,
        GRAB_CENTER_Y - CENTER_TOLERANCE_Y,
        CENTER_TOLERANCE_X * 2,
        CENTER_TOLERANCE_Y * 2,
        color=(255, 0, 255),
        thickness=2,
    )
    if label_y < 0:
        label_y = GRAB_CENTER_Y + 10
    osd_img.draw_string_advanced(
        label_x,
        label_y,
        20,
        "grab center (%d,%d)" % (GRAB_CENTER_X, GRAB_CENTER_Y),
        color=(255, 0, 255),
    )


class BallDetectModel(AIBase):
    def __init__(
        self,
        kmodel_path,
        labels,
        confidence_threshold,
        model_input_size=BALL_MODEL_INPUT_SIZE,
        nms_threshold=NMS_THRESH,
        max_boxes_num=MAX_BOXES,
        rgb888p_size=RGB888P_SIZE,
        display_size=None,
        debug_mode=0,
    ):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [align_up(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [align_up(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.last_count = 0
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("ball_preprocess_cfg", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = letterbox_pad_param(
                self.rgb888p_size,
                self.model_input_size,
            )
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        with ScopedTiming("ball_postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )
            return det_res

    def collect_detections(self, dets):
        detections = []
        self.last_count = 0
        if not dets:
            return detections
        if len(dets) < 3 or len(dets[0]) <= 0:
            return detections

        for i in range(len(dets[0])):
            x, y, w, h = map(lambda v: int(round(v, 0)), dets[0][i])
            cls_id = int(dets[1][i])
            score = float(dets[2][i])

            if cls_id != YELLOW_CLASS_ID:
                continue
            if not pass_ball_shape_filter(x, y, w, h, self.display_size):
                continue
            if cls_id < 0 or cls_id >= len(self.labels):
                label = str(cls_id)
            else:
                label = self.labels[cls_id]

            x, y = clamp_xy(x, y, self.display_size[0], self.display_size[1])
            if x + w > self.display_size[0]:
                w = self.display_size[0] - x
            if y + h > self.display_size[1]:
                h = self.display_size[1] - y
            if w <= 0 or h <= 0:
                continue

            detections.append({
                "label": label,
                "score": score,
                "box": [x, y, w, h],
                "display_size": self.display_size,
            })

        detections.sort(key=lambda d: d["score"], reverse=True)
        self.last_count = len(detections)
        return detections


def import_mqtt_client():
    try:
        from umqtt.simple import MQTTClient
        return MQTTClient
    except Exception:
        pass
    try:
        from simple import MQTTClient
        return MQTTClient
    except Exception:
        pass
    try:
        from mqtt import MQTTClient
        return MQTTClient
    except Exception:
        return None


def mqtt_bytes(value):
    if isinstance(value, bytes):
        return value
    return str(value).encode()


def import_socket_module():
    try:
        import usocket as socket
        return socket
    except Exception:
        pass
    try:
        import socket
        return socket
    except Exception:
        return None


def mqtt_tcp_reachable(host, port):
    socket = import_socket_module()
    sock = None

    if socket is None:
        return True

    try:
        try:
            socket.setdefaulttimeout(MQTT_SOCKET_TIMEOUT_S)
        except Exception:
            pass

        addr = socket.getaddrinfo(host, port)[0][-1]
        sock = socket.socket()
        try:
            sock.settimeout(MQTT_SOCKET_TIMEOUT_S)
        except Exception:
            pass

        sock.connect(addr)
        return True
    except Exception as e:
        print("mqtt broker unreachable:", host, port, e)
        return False
    finally:
        if sock:
            try:
                sock.close()
            except Exception:
                pass


def compact_mqtt_payload(payload):
    return {
        "device": payload.get("device"),
        "ts_ms": payload.get("ts_ms"),
        "found": payload.get("found"),
        "center": payload.get("center"),
        "box": payload.get("box"),
        "area": payload.get("area"),
        "offset": payload.get("offset"),
        "center_ok": payload.get("center_ok"),
        "area_ok": payload.get("area_ok"),
        "stable_count": payload.get("stable_count"),
        "grab_ready": payload.get("grab_ready"),
        "fps": payload.get("fps"),
    }


def mqtt_set_socket_timeout(client):
    try:
        if client.sock:
            client.sock.settimeout(MQTT_SOCKET_TIMEOUT_S)
    except Exception:
        pass


def diag_print(*args):
    if MQTT_DIAG_LOG:
        print(*args)


def wlan_ifconfig(wlan):
    try:
        return wlan.ifconfig()
    except Exception:
        return None


def connect_wifi_if_needed():
    if WIFI_SSID == "":
        diag_print("wifi check skipped: empty ssid")
        return True

    try:
        import network
    except Exception as e:
        print("wifi disabled:", e)
        return False

    try:
        wlan = network.WLAN(network.STA_IF)
    except Exception:
        wlan = network.WLAN(0)

    try:
        if wlan.isconnected():
            diag_print("wifi already connected:", wlan_ifconfig(wlan))
            return True
    except Exception:
        pass

    try:
        diag_print("wifi connect start:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < WIFI_TIMEOUT_MS:
            try:
                if wlan.isconnected():
                    print("wifi connected:", wlan.ifconfig())
                    return True
            except Exception:
                pass
            time.sleep_ms(200)
        diag_print("wifi connect timeout:", WIFI_SSID, wlan_ifconfig(wlan))
    except Exception as e:
        print("wifi connect failed:", e)

    return False


def mqtt_connect():
    if not ENABLE_MQTT:
        diag_print("mqtt disabled: ENABLE_MQTT is False")
        return None
    if MQTT_BROKER == "":
        print("mqtt disabled: empty broker")
        return None
    diag_print("mqtt connect start:", MQTT_BROKER, MQTT_PORT)
    if not connect_wifi_if_needed():
        diag_print("mqtt connect skipped: wifi not connected")
        return None
    if not mqtt_tcp_reachable(MQTT_BROKER, MQTT_PORT):
        return None

    MQTTClient = import_mqtt_client()
    if MQTTClient is None:
        print("mqtt disabled: MQTTClient not found")
        return None

    try:
        client = MQTTClient(
            mqtt_bytes(MQTT_DEVICE_ID),
            MQTT_BROKER,
            MQTT_PORT,
            MQTT_USER,
            MQTT_PASSWORD,
            MQTT_KEEPALIVE,
        )
        client.connect()
        mqtt_set_socket_timeout(client)
        print("mqtt connected:", MQTT_BROKER, MQTT_PORT)
        return client
    except Exception as e:
        print("mqtt connect failed:", e)
        return None


def mqtt_publish(client, payload):
    client.publish(mqtt_bytes(MQTT_TOPIC), mqtt_bytes(ujson.dumps(compact_mqtt_payload(payload))))


def main():
    pl = None
    model = None
    mqtt_client = None
    calibration_log = None
    last_log_ms = 0
    calibration_sample_index = 1
    frame_count = 0
    last_mqtt_ms = 0
    last_mqtt_retry_ms = 0
    stable_count = 0
    last_seen_ms = 0

    try:
        pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode=DISPLAY_MODE)
        pl.create()
        display_size = pl.get_display_size()

        model = BallDetectModel(
            BALL_MODEL_PATH,
            BALL_LABELS,
            BALL_CONF,
            model_input_size=BALL_MODEL_INPUT_SIZE,
            display_size=display_size,
            debug_mode=DEBUG_MODE,
        )
        model.config_preprocess()
        if MQTT_CONNECT_AT_START:
            mqtt_client = mqtt_connect()
        calibration_log = open_calibration_log()
        calibration_sample_index = next_calibration_sample_index()
        print("calibration sample start:", calibration_sample_index)
        clock = time.clock()

        while True:
            os.exitpoint()
            clock.tick()
            now = time.ticks_ms()
            fps = clock.fps()
            frame_count += 1

            with ScopedTiming("ball_center_detect", DEBUG_MODE > 0):
                img = pl.get_frame()
                pl.osd_img.clear()
                draw_grab_center(pl.osd_img)

                dets = model.run(img)
                balls = model.collect_detections(dets)
                best_ball = balls[0] if balls else None

                if best_ball:
                    draw_ball(pl.osd_img, best_ball)
                    preview_payload = build_ball_event(best_ball, now, stable_count, fps)
                    if preview_payload["frame_ready"]:
                        stable_count += 1
                    else:
                        stable_count = 0
                    last_seen_ms = now
                    payload = build_ball_event(best_ball, now, stable_count, fps)
                else:
                    if last_seen_ms != 0 and time.ticks_diff(now, last_seen_ms) <= BALL_LOST_TIMEOUT_MS:
                        payload = empty_event(now, display_size, stable_count, fps)
                    else:
                        stable_count = 0
                        payload = empty_event(now, display_size, stable_count, fps)

                status = "fps %.1f ball:%d ready:%d/%d" % (
                    fps,
                    model.last_count,
                    stable_count,
                    BALL_STABLE_FRAME_MIN,
                )
                pl.osd_img.draw_string_advanced(6, 6, 22, status, color=(255, 255, 255))
                pl.show_image()

                if ENABLE_JSON_LOG and time.ticks_diff(now, last_log_ms) >= LOG_INTERVAL_MS:
                    # print(ujson.dumps(payload))
                    last_log_ms = now

                if calibration_log is not None and (CALIBRATION_LOG_EVERY_N_FRAMES <= 1 or frame_count % CALIBRATION_LOG_EVERY_N_FRAMES == 0):
                    if should_write_calibration_sample(payload):
                        image_name = save_calibration_image(pl, img, calibration_sample_index, payload)
                        write_calibration_log(calibration_log, payload, calibration_sample_index, image_name)
                        calibration_sample_index += 1

                if ENABLE_MQTT and time.ticks_diff(now, last_mqtt_ms) >= MQTT_INTERVAL_MS:
                    if mqtt_client is None and time.ticks_diff(now, last_mqtt_retry_ms) >= MQTT_RECONNECT_MS:
                        mqtt_client = mqtt_connect()
                        last_mqtt_retry_ms = now
                    if mqtt_client is not None:
                        try:
                            mqtt_publish(mqtt_client, payload)
                        except Exception as e:
                            print("mqtt publish failed:", e)
                            mqtt_client = None
                            last_mqtt_retry_ms = now
                    last_mqtt_ms = now

                gc.collect()

    except KeyboardInterrupt:
        print("stopped")
    except BaseException as e:
        sys.print_exception(e)
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        if model:
            try:
                model.deinit()
            except Exception:
                pass
        if pl:
            try:
                pl.destroy()
            except Exception:
                pass
        if mqtt_client:
            try:
                mqtt_client.disconnect()
            except Exception:
                pass
        if calibration_log:
            try:
                calibration_log.close()
            except Exception:
                pass
        gc.collect()
        time.sleep(1)


if __name__ == "__main__":
    main()
