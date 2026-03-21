"""Multi-Layer Perceptron class."""

import torch


class MLP(torch.nn.Module):
    """MLP model."""

    def __init__(
        self,
        in_features,
        hidden_features,
        out_features,
        activation=None,
        bias=True,
        activate_final=False,
    ):
        """Create MLP module.

        Args:
            in_features: Number of features of the input.
            hidden_features: Iterable of ints. Output sizes of the layers.
            activation: Activation function. If None, defaults to ReLU.
            bias: Bool. Whether to use bias.
            activate_final: Bool. Whether to apply activation function to the
                final output.
        """
        super(MLP, self).__init__()

        self._in_features = in_features
        self._hidden_features = hidden_features
        self._out_features = out_features
        self.bias = bias
        
        if activation is None:
            activation = torch.nn.ReLU()
        self.activation = activation

        features_list = [in_features] + list(hidden_features) + [out_features]
        module_list = []
        for i in range(len(features_list) - 1):
            if i > 0:
                module_list.append(activation)
            layer = torch.nn.Linear(
                in_features=features_list[i],
                out_features=features_list[i + 1],
                bias=bias,
            )
            module_list.append(layer)

        if activate_final:
            module_list.append(activation)

        self.net = torch.nn.Sequential(*module_list)

    def forward(self, x):
        """Apply MLP to input.

        Args:
            x: Tensor of shape [batch_size, ..., in_features].

        Returns:
            Output of shape [batch_size, ..., self.out_features]. If
                self._apply_to_last_dim, then an arbitrary number of
                intermediate dimensions will be preserved.
        """

        return self.net(x)

    @property
    def in_features(self):
        return self._in_features

    @property
    def hidden_features(self):
        return self._hidden_features

    @property
    def out_features(self):
        return self._out_features
