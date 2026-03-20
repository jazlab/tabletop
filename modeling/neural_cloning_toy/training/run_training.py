"""Run model evaluation."""

import importlib
import json
import logging
import os
import sys
import numpy as np
import torch

from absl import app, flags

sys.path.append("..")
from python_utils.configs import override_config
from python_utils.configs import build_from_config

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "config",
    # 'configs.cursor_control_teacher',
    'configs.cursor_control_student',
    # 'configs.arm_2dof_teacher',
    "Module name of task config to use.",
)
flags.DEFINE_string(
    "config_overrides",
    "",
    "JSON-serialized config overrides. This is typically not used locally, "
    "only when running sweeps on Openmind.",
)
flags.DEFINE_string("log_directory", "logs", "Prefix for the log directory.")
flags.DEFINE_string(
    "metadata",
    "",
    "Metadata to write to metadata.log file. Often used for slurm task ID.",
)


def main(_):

    ############################################################################
    # Load config
    ############################################################################

    config_module = importlib.import_module(FLAGS.config)
    config = config_module.get_config()
    logging.info(FLAGS.config_overrides)

    # Apply config overrides
    config = override_config.override_config_from_json(
        config, FLAGS.config_overrides
    )

    ############################################################################
    # Create logging directory
    ############################################################################

    log_dir = FLAGS.log_directory
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # If log_dir is not empty, create a new enumerated sub-directory in it for
    # logging.
    list_log_dir = os.listdir(log_dir)
    if (
        len(list_log_dir) != 0
    ):  # For safety, explicitly use len instead of bool
        existing_log_subdirs = [
            int(filename) for filename in list_log_dir if filename.isdigit()
        ]
        if not existing_log_subdirs:
            existing_log_subdirs = [-1]
        new_log_subdir = str(max(existing_log_subdirs) + 1)
        log_dir = os.path.join(log_dir, new_log_subdir)
        os.mkdir(log_dir)

    logging.info("Log directory: {}".format(log_dir))

    ############################################################################
    # Log config name, config overrides, config, and metadata
    ############################################################################

    def _log(log_filename, thing_to_log):
        f_name = os.path.join(log_dir, log_filename)
        logging.info("In file {} will be written:".format(log_filename))
        logging.info(thing_to_log)
        json.dump(thing_to_log, open(f_name, "w"))

    _log("config_name.json", FLAGS.config)
    _log("config_overrides.json", FLAGS.config_overrides)
    _log("config.json", config)
    _log("metadata.json", FLAGS.metadata)

    ############################################################################
    # Set random seed and run trainer
    ############################################################################

    random_seed = config["random_seed"]
    np.random.seed(config["random_seed"])
    torch.manual_seed(config["random_seed"])
    json.dump(random_seed, open(os.path.join(log_dir, "random_seed.json"), "w"))
    trainer = build_from_config.build_from_config(config)
    trainer(log_dir)


if __name__ == "__main__":
    app.run(main)
