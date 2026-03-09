import flask
import cv2
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
import os

from auto_pose.ae import factory, utils

app = flask.Flask(__name__)

# --- Load AAE Model ---
workspace_path = os.environ.get('AE_WORKSPACE_PATH')
experiment_name = 'my_autoencoder'
experiment_group = 'exp_group' 

log_dir = utils.get_log_dir(workspace_path, experiment_name, experiment_group)
ckpt_dir = utils.get_checkpoint_dir(log_dir)
codebook, dataset = factory.build_codebook_from_name(experiment_name, experiment_group, return_dataset=True)

gpu_options = tf.GPUOptions(allow_growth=True, per_process_gpu_memory_fraction=0.5)
sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
factory.restore_checkpoint(sess, tf.train.Saver(), ckpt_dir)

@app.route('/get_rotation', methods=['POST'])
def get_rotation():
    # Receive image from YOLO node
    file = flask.request.files['image'].read()
    nparr = np.frombuffer(file, np.uint8)
    im = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    # AAE preprocessing
    im = cv2.resize(im, (128, 128))
    
    # Get Rotation Matrix
    R = codebook.nearest_rotation(sess, im)
    
    print("Inference successful. R Matrix sent.")
    return flask.jsonify({"R": R.tolist()})

if __name__ == '__main__':
    # Running on port 5001 to avoid common conflicts
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=False)
