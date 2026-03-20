"""Trainer class for models."""

import json
import logging
from pathlib import Path

import torch


class Trainer:

    def __init__(
        self,
        model,
        training_steps,
        batch_size,
        lr=0.001,
        optimizer=torch.optim.Adam,
        grad_clip=1,
        num_log_steps=10,
    ):
        """Trainer constructor."""
        self._model = model
        self._training_steps = training_steps
        self._batch_size = batch_size
        self._grad_clip = grad_clip
        self._scalar_eval_every = self._training_steps // num_log_steps
        self._task = self._model.task

        # Create optimizers
        self._optimizer = optimizer(self._model.parameters(), lr=lr)

    def step_optimizers(self):
        """Step optimizers."""
        self._optimizer.zero_grad()
        loss = self._model.loss(self._batch_size, test=False)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self._model.parameters(), self._grad_clip)
        self._optimizer.step()
        return loss.item()
    
    def eval(self):
        stats = {}
        for mode, test in zip(["train", "test"], [False, True]):
            loss = self._model.loss(self._batch_size, test=test)
            stats[f"{mode.capitalize()} loss"] = loss.item()
        return stats

    def cache(self, log_dir, **for_json_dump):
        """Cache model outputs."""
        print(f"Saving outputs to {log_dir}")
        self._model.cache(write_dir=log_dir)
        for key, value in for_json_dump.items():
            json.dump(value, open(f"{log_dir}/{key}.json", "w"))

    def __call__(self, log_dir):
        """Evaluate on the current task."""
        logging.info("\nBeginning training")
        log_dir = Path(log_dir)
        losses = []
        stats = []
        for step in range(self._training_steps):
            loss = self.step_optimizers()
            losses.append(loss)

            if step % self._scalar_eval_every == 0:
                step_stats = self.eval()
                step_stats["step"] = step
                step_stats["loss"] = loss
                stats.append(step_stats)
                log_str = (
                    f"    Step {step} / {self._training_steps}; " +
                    ", ".join([f"{k} = {v:.7f}" for k, v in step_stats.items()])
                )
                logging.info(log_str)
        self.cache(log_dir, losses=losses, stats=stats)
