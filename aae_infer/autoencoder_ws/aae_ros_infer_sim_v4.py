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

# === Configuration ===
workspace_path = os.environ.get('AE_WORKSPACE_PATH')
if workspace_path is None:
    raise EnvironmentError("Please set AE_WORKSPACE_PATH environment variable")

experiment_name = 'my_autoencoder'
experiment_group = 'exp_group'

# === Load Model (once at startup) ===
print(f"[{datetime.now()}] Loading AutoEncoder codebook...")
log_dir = utils.get_log_dir(workspace_path, experiment_name, experiment_group)
ckpt_dir = utils.get_checkpoint_dir(log_dir)

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
        "requests_processed": request_count,
        "avg_inference_time_s": round(avg, 3)
    })


if __name__ == '__main__':
    print("Starting Flask server with threaded=True (concurrent requests enabled)")
    print("Access health check at http://localhost:5001/health")
    app.run(
        host='0.0.0.0',
        port=5001,
        debug=False,
        threaded=True,        # ← Critical fix for continuous operation
        use_reloader=False    # Prevent double-loading in some environments
    )
