"""Teacher model."""

import copy
import torch
from pathlib import Path
from models import mlp as mlp_lib
import json
from python_utils.configs import build_from_config


class Teacher(torch.nn.Module):
    """Teacher model."""

    def __init__(
        self,
        task,
        hidden_features,
        action_scale=1.0,
    ):
        """Create Teacher model."""
        super(Teacher, self).__init__()
        self._task = task
        self._action_scale = action_scale
        self._net = mlp_lib.MLP(
            in_features=task.observation_size,
            hidden_features=hidden_features,
            out_features=task.action_size,
        )

    def forward(self, state, goal):
        """Apply net to input."""
        observation = self._task.get_observation(state, goal)
        net_output = self._net(observation)
        action = self._action_scale * torch.nn.functional.tanh(net_output)
        return action
    
    def loss(self, batch_size, test):
        state, goal = self._task.get_batch(batch_size, test=test)
        model_output = self.forward(state, goal)
        target = self._task.get_target_action(state, goal)
        
        # MSE loss
        # loss = torch.nn.functional.mse_loss(model_output, target)

        # Angle loss
        target_normalized = target / target.norm(dim=-1, keepdim=True)
        model_output_normalized = model_output / model_output.norm(dim=-1, keepdim=True)
        loss = torch.nn.functional.mse_loss(model_output_normalized, target_normalized)

        return loss

    def cache(self, write_dir):
        """Cache model outputs."""
        torch.save(self.state_dict(), write_dir / "model_snapshot.pth")

    @property
    def task(self):
        """Return task."""
        return self._task

    @property
    def net(self):
        """Return network."""
        return self._net
    
    @property
    def action_scale(self):
        """Return action scale."""
        return self._action_scale
    

class ActivationHook():
    """ Hook to capture model activations."""
    
    def __init__(self):
        self.activation = None

    def __call__(self, module, input, output):
        del module
        del input
        self.activation = output


class Student(Teacher):

    def __init__(self, teacher_log_dir, reg_coeff, **kwargs):
        self._reg_coeff = reg_coeff
        teacher_log_dir = Path(teacher_log_dir).resolve()
        config_path = teacher_log_dir / "config.json"
        model_snapshot_path = teacher_log_dir / "model_snapshot.pth"
        config = json.load(open(config_path))
        model_config = config["kwargs"]["model"]
        model_kwargs = copy.deepcopy(model_config["kwargs"])
        model_kwargs.update(kwargs)
        super().__init__(**model_kwargs)
        self._teacher_model = build_from_config.build_from_config(model_config)
        self._teacher_model.load_state_dict(torch.load(model_snapshot_path))

        # Add activation hooks
        self._activation_hook_teacher = ActivationHook()
        self._activation_hook_student = ActivationHook()
        self._teacher_model.net.net[-1].register_forward_hook(self._activation_hook_teacher)
        self.net.net[-1].register_forward_hook(self._activation_hook_student)

    def loss_regularization(self, batch_size, test):
        """Compute the regularization loss."""
        state, goal = self._task.get_batch(batch_size, test=test)
        self._teacher_model(state, goal)
        self(state, goal)
        activation_teacher = self._activation_hook_teacher.activation
        activation_student = self._activation_hook_student.activation
        loss = torch.nn.functional.mse_loss(activation_student, activation_teacher)
        # print(f"loss raw: {loss.item()}")
        return self._reg_coeff * loss

    def loss(self, batch_size, test):
        loss_action = super().loss(batch_size, test)
        loss_regularization = self.loss_regularization(batch_size, test)
        # print(f"loss_regularization: {loss_regularization.item()}")
        return loss_action + loss_regularization
