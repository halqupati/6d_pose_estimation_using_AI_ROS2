import flask
import cv2
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
import os
import time
import threading
from datetime import datetime
from auto_pose.ae import factory, utils

app = flask.Flask(__name__)


def get_env_int(name, default):
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    try:
        return int(raw_value)
    except ValueError:
        print(f"[{datetime.now()}] Invalid integer for {name}={raw_value!r}; using {default}")
        return default

# === Configuration ===
workspace_path = os.environ.get('AE_WORKSPACE_PATH')
if workspace_path is None:
    raise EnvironmentError("Please set AE_WORKSPACE_PATH environment variable")

# Explicit nut-model selection. Each key maps to exactly one AAE experiment; there is
# deliberately NO fallback between entries (selecting m6 never loads the M20 model).
#   m20: legacy experiment trained on model/one_bl.ply (30 mm across flats x 16 mm)
#   m6 : DIN 934 M6 experiment, model/nut_m6.ply (10 mm across flats x 5 mm)
NUT_MODEL_REGISTRY = {
    "m20": {"experiment_name": "my_autoencoder", "experiment_group": "exp_group"},
    "m6": {"experiment_name": "my_autoencoder_m6", "experiment_group": "exp_group"},
}
_nut_model_env = os.environ.get("AAE_NUT_MODEL")
nut_model = (_nut_model_env or "m20").strip().lower()
if _nut_model_env is None:
    print(f"[{datetime.now()}] AAE_NUT_MODEL not set; using 'm20' (previous default behaviour).")
if nut_model not in NUT_MODEL_REGISTRY:
    raise SystemExit(
        f"[AAE] Unknown AAE_NUT_MODEL={nut_model!r}. Valid values: {sorted(NUT_MODEL_REGISTRY)}"
    )

experiment_name = NUT_MODEL_REGISTRY[nut_model]["experiment_name"]
experiment_group = NUT_MODEL_REGISTRY[nut_model]["experiment_group"]
server_host = os.environ.get("AAE_SERVER_HOST", "0.0.0.0")
server_port = get_env_int("AAE_SERVER_PORT", 5001)
health_host = os.environ.get("AAE_HEALTH_HOST", "localhost")


def validate_aae_experiment(workspace, exp_name, exp_group, model_key):
    """Fail fast, with an actionable message, before any TF graph is built.

    Checks: experiment cfg exists, a real TF checkpoint (index + data shards) exists,
    the checkpoint's variables belong to this experiment's scope, the codebook
    embedding was actually computed (ae_embed), and its latent size matches the cfg.
    TensorBoard event files are never accepted as weights.
    """
    import configparser
    import glob

    log_dir_ = utils.get_log_dir(workspace, exp_name, exp_group)
    ckpt_dir_ = utils.get_checkpoint_dir(log_dir_)
    cfg_path = utils.get_train_config_exp_file_path(log_dir_, exp_name)
    prefix = f"[AAE:{model_key}:{exp_name}]"

    def fail(msg):
        raise SystemExit(f"{prefix} MODEL NOT LOADABLE: {msg}")

    if not os.path.isfile(cfg_path):
        fail(f"experiment config not found: {cfg_path}")
    cfg = configparser.ConfigParser()
    cfg.read(cfg_path)

    mesh_path = cfg.get("Paths", "MODEL_PATH", fallback="")
    if mesh_path and not os.path.isfile(mesh_path):
        # Not needed for codebook inference, but report it so cfg/model drift is visible.
        print(f"{prefix} WARNING: cfg MODEL_PATH does not exist on this machine: {mesh_path}")

    if not os.path.isdir(ckpt_dir_):
        fail(f"checkpoint directory missing: {ckpt_dir_}")

    state = tf.train.get_checkpoint_state(ckpt_dir_)
    if not state or not state.model_checkpoint_path:
        present = sorted(os.listdir(ckpt_dir_))
        only_events = present and all(name.startswith("events.out.tfevents") for name in present)
        detail = (
            "directory only contains TensorBoard event logs (training was interrupted before the "
            "first SAVE_INTERVAL); these are NOT model weights"
            if only_events
            else f"no TF 'checkpoint' index file. Found: {present or 'nothing'}"
        )
        fail(f"no trained checkpoint in {ckpt_dir_}: {detail}. Train with "
             f"`ae_train {exp_group}/{exp_name}` and then run `ae_embed {exp_group}/{exp_name}`.")

    ckpt_path = state.model_checkpoint_path
    if not os.path.isabs(ckpt_path):
        ckpt_path = os.path.join(ckpt_dir_, ckpt_path)
    if not os.path.isfile(ckpt_path + ".index") or not glob.glob(ckpt_path + ".data-*"):
        fail(f"checkpoint index references {ckpt_path} but its .index/.data-* files are missing")

    try:
        variables = dict(tf.train.list_variables(ckpt_path))
    except Exception as exc:  # corrupted / truncated checkpoint
        fail(f"cannot read checkpoint {ckpt_path}: {exc}")

    scope = exp_name + "/"
    if not any(name.startswith(scope) for name in variables):
        scopes = sorted({name.split("/")[0] for name in variables})
        fail(f"checkpoint variables are under scope(s) {scopes}, expected '{exp_name}'. "
             "This checkpoint belongs to a different experiment.")

    embed_name = scope + "embedding_normalized"
    if embed_name not in variables:
        fail(f"'{embed_name}' missing from checkpoint; run `ae_embed {exp_group}/{exp_name}`.")
    embedding = tf.train.load_variable(ckpt_path, embed_name)
    if not np.any(embedding):
        fail("codebook embedding is all zeros (trained but never embedded). "
             f"Run `ae_embed {exp_group}/{exp_name}`.")

    latent = cfg.getint("Network", "LATENT_SPACE_SIZE", fallback=None)
    if latent is not None and embedding.shape[1] != latent:
        fail(f"embedding latent size {embedding.shape[1]} != cfg LATENT_SPACE_SIZE {latent}")

    print(f"{prefix} checkpoint OK: {ckpt_path} | codebook {embedding.shape[0]} views x {embedding.shape[1]} dims")
    return ckpt_dir_


# === Load Model (once at startup) ===
print(f"[{datetime.now()}] Selected nut model: {nut_model} -> experiment {experiment_group}/{experiment_name}")
ckpt_dir = validate_aae_experiment(workspace_path, experiment_name, experiment_group, nut_model)
print(f"[{datetime.now()}] Loading AutoEncoder codebook...")

codebook, dataset = factory.build_codebook_from_name(
    experiment_name, experiment_group, return_dataset=True
)

# GPU memory settings - conservative to avoid growth over time
gpu_options = tf.GPUOptions(
    allow_growth=True,
    per_process_gpu_memory_fraction=0.5  # Reduced from 0.6
)
sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
saver = tf.train.Saver()
factory.restore_checkpoint(sess, saver, ckpt_dir)
print(f"[{datetime.now()}] Model loaded successfully from {ckpt_dir}")

# === Stats ===
lock = threading.Lock()
request_count = 0
total_inference_time = 0.0


@app.route('/get_rotation', methods=['POST'])
def get_rotation():
    global request_count, total_inference_time

    start_time = time.time()
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    try:
        print(f"[{timestamp}] Request #{request_count+1} received")

        # Receive image
        if 'image' not in flask.request.files:
            return flask.jsonify({"error": "No image part in request"}), 400

        file_bytes = flask.request.files['image'].read()
        print(f"[{timestamp}] Received {len(file_bytes)} bytes")

        # Decode
        nparr = np.frombuffer(file_bytes, np.uint8)
        im = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if im is None:
            print(f"[{timestamp}] ERROR: cv2.imdecode failed (corrupted image?)")
            return flask.jsonify({"error": "Failed to decode image"}), 400

        print(f"[{timestamp}] Decoded shape: {im.shape}")

        # Resize to model input size
        im_resized = cv2.resize(im, (128, 128))
        print(f"[{timestamp}] Resized to: {im_resized.shape}")

        # === Critical: AutoEncoder inference ===
        infer_start = time.time()
        R = codebook.nearest_rotation(sess, im_resized)
        infer_duration = time.time() - infer_start

        print(f"[{timestamp}] nearest_rotation took {infer_duration:.3f} seconds")

        # Update stats
        with lock:
            request_count += 1
            total_inference_time += infer_duration
            avg_time = total_inference_time / request_count

        print(f"[{timestamp}] Success | Avg inference: {avg_time:.3f}s | Total requests: {request_count}")

        return flask.jsonify({"R": R.tolist()})

    except Exception as e:
        import traceback
        print(f"[{timestamp}] ERROR in /get_rotation:")
        traceback.print_exc()
        return flask.jsonify({"error": str(e)}), 500

    finally:
        total_time = time.time() - start_time
        # Optional: warn if full request takes too long
        if total_time > 2.0:
            print(f"[{timestamp}] WARNING: Full request took {total_time:.2f}s")


@app.route('/health', methods=['GET'])
def health():
    with lock:
        avg = total_inference_time / request_count if request_count > 0 else 0
    return flask.jsonify({
        "status": "healthy",
        "nut_model": nut_model,
        "experiment": f"{experiment_group}/{experiment_name}",
        "requests_processed": request_count,
        "avg_inference_time_s": round(avg, 3)
    })


if __name__ == '__main__':
    print("Starting Flask server with threaded=True (concurrent requests enabled)")
    print(f"Access health check at http://{health_host}:{server_port}/health")
    app.run(
        host=server_host,
        port=server_port,
        debug=False,
        threaded=True,        # ← Critical fix for continuous operation
        use_reloader=False    # Prevent double-loading in some environments
    )
